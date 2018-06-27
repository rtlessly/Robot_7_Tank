/*****************************************************************************
 Robot_7_Tank
 
 Robot on "tank" chassis with ultrasonic and IR object avoidance, IR step sensor, and IMU.

 * Uses OSEPP Tri-Tank chassis in "long tank" configuration
 * Uses Adafruit motor controller shield to control motors
 * Ultrasonic sensor (aka sonar) scans for objects ahead in path
 * Sonar is mounted on a servo to allow it to be pivoted left and right to scan
   for best alternate route when object encountered
 * IR proximity sensors detect objects to the side; robot turns away from obstacle
   until no longer detected
 * Another IR proximity sensor is used as a forward step sensor
 * IMU used to measure robot orientation and measure turn angles
 * Simple IR sensor to turn motors on/off via IR remote
 
 created April 2018
 by R. Terry Lessly
 ****************************************************************************/
#include <Arduino.h>
#include <avr/wdt.h>
#include <Servo.h>
#include <RTL_Stdlib.h>
#include <RTL_EventFramework.h>
#include <RTL_I2C.h>
#include <AF_MotorShield2.h>
#include <AF_DCMotor2.h>
#include <IRSensor.h>
#include <IRProximitySensor.h>
#include <SonarSensor.h>
#include "RTL_IMU_Razor\RTL_IMU.h"

#define DEBUG 0

//******************************************************************************
// Forward declarations
//******************************************************************************
bool CheckRemoteCommand();
bool CheckStepSensor();
bool CheckLeftRightProxSensor();
bool CheckSonarSensor();
void Stop();
void GoForward();
void GoBackward();
void GoBackward(uint32_t duration);
void GoBackward(uint32_t duration, bool (*predicate)());
void Spin(char direction);
void Spin(char direction, uint32_t duration);
void Spin180();
void SetMotors(int leftSpeed, int rightSpeed);
uint16_t Ping();
char sonarScan();
void BlinkLEDCount(uint16_t count, uint16_t onTime = 100, uint16_t offTime = 100);
void IndicateFailure(const __FlashStringHelper* msg=nullptr);

//******************************************************************************
// Constants
//******************************************************************************
const int servoPin = 9;             // Servo on Arduino pin 9
const int servoBias = 112;          // Set servo center position bias to compensate for the servo orientation
const int maxSpeed = 255;           // Maximum motor speed
const int cruiseSpeed = 180;        // Normal speed to run the motors
const int threshold = 40;           // Sonar threshold distance in centimeters

//******************************************************************************
// Global variables
//******************************************************************************
AF_MotorShield2 motorController;
AF_DCMotor2 leftMotor(motorController, 0);
AF_DCMotor2 rightMotor(motorController, 1);

IRSensor ir(3);                     // IR Remote Sensor on pin 3 (for receiving signals from IR remote)
SonarSensor sonar(2, 4);            // Ultrasonic sensor on Arduino pins 2 and 4
IRProximitySensor proxRight(5, 3);  // Right IR Proximity sensor on pin 5 (for obstacle detection on the right side)
IRProximitySensor proxLeft(6, 3);   // Left IR Proximity sensor on pin 6 (for obstacle detection on the left side)
IRProximitySensor proxStep(7);      // Step IR Proximity sensor on pin 7 (for step, i.e. drop-off, detection)
Servo panServo;                     // For panning the ultrasonic sensor left and right

int HW_LED_PIN = 13;                // Hardware LED pin

bool motorsEnabled = false;         // Indicates if the motors are enabled

#if DEBUG
uint16_t loopCount = 0;             // Counts iterations through loop() method - for debugging
#endif



//******************************************************************************
// Arduino setup method
// 
// Performs initialization
//******************************************************************************
void setup()
{
    Serial.begin(57600);
    Wire.begin();

    // Initialize AF Motor Shield
    motorController.Begin();

    // Configure Servo for Sonar sensor on Arduino pin 9
    panServo.attach(servoPin);

    // Center servo to point straight ahead
    panServo.write(servoBias);

    // Validate presence of IMU
    byte imuID = 0;
    byte imuReady = 0;
    byte i2cStatus = 0;
    byte command[2] = { '#', 0 };

    // Validate existence of IMU by checking for device ID
    while (true) 
    {
        command[1] = CMD_ID;
        i2cStatus = I2C_SendMessage(RAZOR_IMU_ADDRESS, command, sizeof(command), &imuID, sizeof(imuID));

        if (i2cStatus == 0 && imuID == RAZOR_IMU_ID) break;

        if (i2cStatus != 0)
            Logger() << F("I2C communication failure with IMU; status=") << i2cStatus << endl;
        else
            Logger() << F("Unable to verify existence of IMU; ID:(expected, actual)=(") << _HEX(RAZOR_IMU_ID) << ',' << _HEX(imuID) << ')' << endl;

        IndicateFailure();
    }

    Logger() << F("IMU ID confirmed: (expected, actual)=(") << _HEX(RAZOR_IMU_ID) << ',' << _HEX(imuID) << ')' << endl;

    // Wait 10 seconds for the IMU to be ready
    auto timeout = millis() + 10000ul;

    while (imuReady == 0 && millis() < timeout)
    {
        //i2cStatus = I2C_SendRequest(RAZOR_IMU_ADDRESS, CMD_IS_READY, imuReady);
        command[1] = CMD_IS_READY;
        i2cStatus = I2C_SendMessage(RAZOR_IMU_ADDRESS, command, sizeof(command), &imuReady, sizeof(imuReady));

        if (i2cStatus != 0)
        {
            Logger() << F("I2C communication failure with IMU; status=") << i2cStatus << endl;
            break;
        }
    }

    // If the IMU failed to report ready this will loop forever blinking "SOS" 
    // on the LED and repeating the message
    while (imuReady == 0)
    {
        IndicateFailure(F("IMU failed to initialize."));
    }

    Logger() << F("IMU ready") << endl;

    // Setup a 4 second watchdog timer to reset microcontroller if program locks up
    wdt_enable(WDTO_4S);
}


//******************************************************************************
// Arduino loop method
//******************************************************************************
void loop()
{
    // The watchdog timer (wdt) will restart the Arduino unless it is reset at least every 4 seconds
    wdt_reset();

    TRACE(Logger() << F("loopCount=") << loopCount++ << endl);

    if (CheckRemoteCommand()) return;

    if (CheckStepSensor()) return;

    if (CheckLeftRightProxSensor()) return;

    if (CheckSonarSensor()) return;

    // If nothing else happened then keep moving forward.
    GoForward();
}


bool CheckRemoteCommand()
{
    // The watchdog timer (wdt) will restart the Arduino unless it is reset at least every 4 seconds
    wdt_reset();

    // The timeout helps to eliminate rapid, repeated triggers (bounce) caused by pulsed signals from IR Remote
    static uint32_t debounceTimeout = 0;

    // Check if the IR Remote Sensor detected a signal
    if (!ir.Read() || (millis() < debounceTimeout)) return false;
        
    TRACE(Logger() << F("Recieved IR Remote signal") << endl);
    Stop();     // If motors are running this ensures they are stopped before being disabled, otherwise it does nothing
    motorsEnabled = !motorsEnabled;
    debounceTimeout = millis() + 500;

    return true;
}


bool CheckStepSensor()
{
    // The watchdog timer (wdt) will restart the Arduino unless it is reset at least every 4 seconds
    wdt_reset();

    // Check if sensor triggered (triggered if sensor returns 0 (false))
    auto stepDetected = !proxStep.Read();

    if (!stepDetected) return false;

    // If step sensor is triggered then stop and back up
    Logger() << F("Step sensor triggered") << endl;
    Stop();
    GoBackward();

    while (!proxStep.Read());   // Backup until step sensor turns off

    GoBackward(500);            // Then back up a little more
    Spin180();                  // And then turn around

    return true;
}


bool CheckLeftRightProxSensor()
{
    // The watchdog timer (wdt) will restart the Arduino unless it is reset at least every 4 seconds
    wdt_reset();

    // Read both left and right sensors
    auto left = proxLeft.Read();
    auto right = proxRight.Read();

    // If neither sensor triggered then exit
    if (!(left || right)) return false;

    // If both sensors triggered then we may be in a corner so turn around
    if (left && right)
    {
        Logger() << F("Obstacle detected by both left and right IR sensors.") << endl;

        // Stop and then backup while both sensors are triggered (or 1 second has elapsed)
        Stop();
        GoBackward(1000, [] { return proxLeft.Read() && proxRight.Read();} );
        GoBackward(500);            // Then back up a little more
        Spin180();                  // And then turn around
    }
    // Else if left proximity sensor triggered then spin to the right to avoid obstacle
    else if (left)
    {
        Logger() << F("Obstacle detected by left IR sensor.") << endl;
        Spin('R');
    }
    // Else right proximity sensor triggered so spin to the left to avoid obstacle
    else
    {
        Logger() << F("Obstacle detected by right IR sensor.") << endl;
        Spin('L');
    }

    return true;
}


bool CheckSonarSensor()
{
    // The watchdog timer (wdt) will restart the Arduino unless it is reset at least every 4 seconds
    wdt_reset();

    // Check for obstacle ahead
    auto ping = sonar.MultiPing();

    if (ping > threshold) return false;
        
    // An obstacle was detected closer than threshold distance
    TRACE(Logger() << F("Obstacle detected ahead by sonar") << endl);
    // Stop when obstacle is detected
    Stop();

    if (ping < threshold / 2)
    {
        GoBackward(500);    // Backup for 1/2 second
    }

    // Try to find a better direction to move
    auto betterDirection = sonarScan();

    // Start a spin in the direction indicated by sonar
    Spin(betterDirection);

    //Keep turning until sonar no longer sees the obstacle
    while (sonar.MultiPing() < threshold)
    {
        delay(50);
    }

    wdt_reset();

    // The obstacle is no longer detected, but it still may not be completely
    // clear of the robot. So continue turning a little longer to hopefully 
    // completely clear it. The time delay depends on the speed of the robot.
    delay(400);
    wdt_reset();

    return true;
}


//***************************************************************************
// Movement control methods.
//***************************************************************************

void Stop()
{
    SetMotors(0, 0);
}


void GoForward()
{
    SetMotors(cruiseSpeed, cruiseSpeed);
}


void GoBackward()
{
    SetMotors(-cruiseSpeed, -cruiseSpeed);
}


void GoBackward(uint32_t duration)
{
    auto timeout = millis() + duration;
    
    GoBackward();

    while (millis() < timeout) wdt_reset();                 // Keep watchdog timer reset to prevent auto-restart

    Stop();
}


void GoBackward(uint32_t duration, bool (*predicate)())
{
    auto timeout = millis() + duration;
    
    GoBackward();

    while (millis() < timeout && predicate()) wdt_reset();  // Keep watchdog timer reset to prevent auto-restart

    Stop();
}


void Spin(char direction)
{
    if (direction == 'R')      // Spin to the right
    {
        SetMotors(-cruiseSpeed, cruiseSpeed);
    }
    else if (direction == 'L') // Spin to the left
    {
        SetMotors(cruiseSpeed, -cruiseSpeed);
    }
}


void Spin(char direction, uint32_t duration)
{
    auto timeout = millis() + duration;

    Spin(direction);

    while (millis() < timeout) wdt_reset();                 // Keep watchdog timer reset to prevent auto-restart

    Stop();
}


void Spin180()
{
    // Tune the spin duration to achieve a 180 degree turn
    Spin('R', 4000);
}


//***************************************************************************
// Set the motor speed
// The speed of both motors is constrained to the range -255 to +255.
//***************************************************************************
void SetMotors(int leftSpeed, int rightSpeed)
{
    if (!motorsEnabled) leftSpeed = rightSpeed = 0;
  
    leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
    rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);
  
    // If the motors don't rotate in the desired direction you can correct it
    // by changing the positive/negative sign of the speed in these calls.
    leftMotor.Run(leftSpeed);
    rightMotor.Run(rightSpeed);
}


//***************************************************************************
// Do one ultrasonic sensor ping.
//***************************************************************************
uint16_t Ping()
{
    uint16_t ping;

    // Keep retrying until we get a good ping (or the watchdog timer resets the board)
    while ((ping = sonar.PingCentimeters()) == PING_FAILED) delay(40);

    return ping;
}


//***************************************************************************
// Use the ultrasonic sensor to scan from right to left. 
// Return whichever direction has more open space (i.e., the direction in which 
// the ultrasonic sensor pings the greatest distances). Return value is 'L' if
// the left side appears more open, or 'R' if the right side appears more open.
//***************************************************************************
char sonarScan()
{
    static const int SCAN_ANGLE = 60;
    
    uint16_t leftscan[SCAN_ANGLE];
    uint16_t rightscan[SCAN_ANGLE];

    panServo.write(servoBias);  // Start sonar at center position looking straight ahead
    delay(100);                 // Wait 100ms for servo to reach start position
    
    ZERO_ARRAY(leftscan);
    ZERO_ARRAY(rightscan);

    // Scan distances from center to max right angle (right side cw)
    for (auto i = 1; i <= SCAN_ANGLE; i++)
    {
        panServo.write(servoBias - i); 
        delay(2);				// Delay 2ms for servo to move to next position
        rightscan[i-1] += Ping();
        wdt_reset();
    }

    // Scan distances from max right angle to center (right side ccw)
    for (auto i = SCAN_ANGLE; i >= 1; i--)
    {
        panServo.write(servoBias - i); 
        delay(2);				// Delay 2ms for servo to move to next position
        rightscan[i-1] += Ping();
        wdt_reset();
    }

    // Scan distances from center to max left (left side ccw)
    for (auto i = 1; i <= SCAN_ANGLE; i++)
    {
        panServo.write(servoBias + i); 
        delay(2);				// Delay 2ms for servo to move to next position
        leftscan[i-1] += Ping();
        wdt_reset();
    }

    // Scan the distance from max left angle to center (left side cw)
    for (auto i = SCAN_ANGLE; i >= 1; i--)
    {
        panServo.write(servoBias + i); 
        delay(2);				// Delay 2ms for servo to move to next position
        leftscan[i-1] += Ping();
        wdt_reset();
    }

    // All done scanning - recenter servo to point straight ahead again
    panServo.write(servoBias);
  
    // Note: Every angle has been scanned twice.
    // Calculate the side that is more open. 
    // A simple way to do this is to just sum the distance for each side.
    // A more sophisticated approach might be to do some sort of statistical 
    // analysis to detect outliers, compute the median, etc.
    uint16_t left = 0;
    uint16_t right = 0;

    for (auto i = 0; i < SCAN_ANGLE; i++) 
    {
        left += leftscan[i];
        right += rightscan[i];
    }

    auto direction = (left > right) ? 'L' : 'R';

    Logger() << F("sonarScan, direction=") << direction << F(", leftSum") << left << F(", rightSum=") << right << endl;
    
    return direction;
}


void BlinkLEDCount(uint16_t count, uint16_t onTime, uint16_t offTime)
{
    for (int i = 0; i < count; i++)
    {
        digitalWrite(HW_LED_PIN, HIGH);
        delay(onTime);
        digitalWrite(HW_LED_PIN, LOW);
        delay(offTime);
    }
}


void IndicateFailure(const __FlashStringHelper* msg)
{
    if (msg != nullptr) Logger() << msg << endl;

    // This flashes "SOS" in morse code
    BlinkLEDCount(3, 200, 150);
    delay(100);
    BlinkLEDCount(3, 500, 150);
    delay(100);
    BlinkLEDCount(3, 200, 150);
    delay(500);
}

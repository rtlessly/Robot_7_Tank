/*****************************************************************************
 * TriTank Robot 01 - Object Avoidance with Sonar Sensor
 * 
 * Robot will avoid objects in front.
 * Ultrasonic sensor scans for the best route to take
 * Robot can't detect objects at it's side, best to install 2 IR Detectors
 *
 * Best used on OSEPP Mechanical Kits.
 * TANK-01
 * TRITANK-01
 *
 * created 08 Nov 2016
 * by Sui Wei
 *
 ****************************************************************************/
 
#include <Arduino.h>
#include <avr/wdt.h>
#include <Servo.h>
#include <RTL_Stdlib.h>
#include <AF_MotorShield2.h>
#include <AF_DCMotor2.h>
#include <IRSensor.h>
#include <IRProximitySensor.h>
#include <SonarSensor.h>
//#include "ultrasonic.h"

#define DEBUG 0


const int servoPin = 9;       // Servo on Arduino pin 9
const int servoBias = 112;    // Set servo center position bias to compensate for the servo orientation
const int maxSpeed = 255;     // Maximum motor speed
const int cruiseSpeed = 180;  // Normal speed to run the motors
const int threshold = 40;     // Sonar threshold distance in centimeters


AF_MotorShield2 motorController;
AF_DCMotor2 leftMotor(motorController, 0);
AF_DCMotor2 rightMotor(motorController, 1);

IRSensor ir(3);                     // IR Remote Sensor on pin 3 (for receiving signals from IR remote)
SonarSensor sonar(2, 4);            // Ultrasonic sensor on Arduino pins 2 and 4
IRProximitySensor proxRight(5, 3);  // Right IR Proximity sensor on pin 5 (for obstacle detection on the right side)
IRProximitySensor proxLeft(6, 3);   // Left IR Proximity sensor on pin 6 (for obstacle detection on the left side)
IRProximitySensor proxStep(7);      // Step IR Proximity sensor on pin 7 (for step, i.e. drop-off, detection)
Servo panServo;                     // For panning the ultrasonic sensor left and right


void setup()
{
    Serial.begin(57600);

    // Initialize AF Motor Shield
    motorController.Begin();

    // Configure Servo for Sonar sensor on Arduino pin 9
    panServo.attach(servoPin);

    // Center servo to point straight ahead
    panServo.write(servoBias);

    // Setup a 4 second watchdog timer to reset microcontroller if program locks up
    wdt_enable(WDTO_4S);
}



bool motorsEnabled = false; // Indicates if the motors are enabled

#if DEBUG
uint16_t loopCount = 0;     // Counts iterations through loop() method - for debugging
#endif


void loop()
{
    TRACE(Logger() << F("loopCount=") << loopCount++ << endl);

    static uint32_t irTimeout = 0;
  
    // The watchdog timer (wdt) will restart the Arduino unless it is reset at least every 4 seconds
    wdt_reset();

    // Check if the IR Remote Sensor detected a signal
    // The timeout helps to eliminate bounce caused by IR Remote sending pulsed signals
    if (ir.Read() && (millis() > irTimeout)) 
    { 
        TRACE(Logger() << F("Recieved IR Remote signal") << endl);
        Stop();     // If motors are running this ensures they are stopped before being disabled, otherwise it does nothing
        motorsEnabled = !motorsEnabled;
        irTimeout = millis() + 500;
    }

    // If step proximity sensor is triggered then stop and reverse direction
    if (!proxStep.Read())
    {
        TRACE(Logger() << F("Step sensor activated") << endl);
        Stop();
        GoBackward();

        while(!proxStep.Read());    // Backup until step sensor turns off

        delay(500);                 // Back up a little more
        wdt_reset();

        // Turn around (tune the delays to achieve 180 degree turn)
        // Note: The turn needs to be executed in multiple steps of less than 4
        //       seconds each to allow the watchdog timer to be periodically reset
        Spin('R', 2000);
        wdt_reset();
        Spin('R', 2000);
        wdt_reset();

        return;
    }

    // Check for obstacle ahead
    auto ping = sonar.MultiPing();

    // Sonar sensor detected an obstacle closer than threshold distance
    if (ping < threshold)
    {
        TRACE(Logger() << F("Obstacle detected by sonar") << endl);
        // Stop when obstacle is detected
        Stop();

        if (ping < threshold/2)
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
    
        return; // Don't continue to the end of the loop() method (i.e., don't start moving just yet)
    }

    // If left proximity sensor is triggered then spin to the right to avoid obstacle
    if (proxLeft.Read())
    {
        TRACE(Logger() << F("Obstacle detected by left IR Sensor") << endl);
        Spin('R');
        return;
    }

    // If right proximity sensor is triggered then spin to the left to avoid obstacle
    if (proxRight.Read())
    {
        TRACE(Logger() << F("Obstacle detected by right IR Sensor") << endl);
        Spin('L');
        return;
    }

    // If there is no problem then keep moving forward.
    GoForward();
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
    GoBackward();
    delay(duration);
    Stop();
    wdt_reset();
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
    Spin(direction);
    delay(duration);
    Stop();
    wdt_reset();
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


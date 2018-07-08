/*******************************************************************************
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
 ******************************************************************************/
#define DEBUG 0

#include <Arduino.h>
#include <avr/wdt.h>
#include <Servo.h>
#include <Wire.h>
#include <RTL_Stdlib.h>
#include <RTL_TaskScheduler.h>
#include <AF_MotorShield2.h>
#include <AF_DCMotor2.h>
#include <IRSensor.h>
#include <IRProximitySensor.h>
#include <SonarSensor.h>
#include "IMU.h"

//******************************************************************************
// Forward declarations
//******************************************************************************
bool CheckRemoteCommand();
bool CheckStepSensor();
bool CheckSideSensors();
bool CheckSonarSensor();
bool CheckCornered();
void Stop();
void GoForward();
void GoBackward();
void GoBackward(uint32_t duration);
void GoBackward(uint32_t duration, bool (*predicate)());
void Spin(char direction);
void Spin(char direction, uint32_t duration);
void Spin180(char direction = 'R');
void PerformMagCalibration();
void SetMotors(int leftSpeed, int rightSpeed);
uint16_t Ping();
char sonarScan();
void BlinkLEDCount(uint16_t count, uint16_t onTime = 100, uint16_t offTime = 100);
void IndicateFailure(const __FlashStringHelper* msg=nullptr);

//******************************************************************************
// Constants
//******************************************************************************
const int LED_PIN = 13;             // Hardware LED pin
const int SERVO_PIN = 9;            // Servo on Arduino pin 9
const int SERVO_BIAS = 112;         // Set servo center position bias to compensate for the servo orientation
const int MAX_SPEED = 255;          // Maximum motor speed
const int CRUISE_SPEED = 180;       // Normal speed to run the motors
const int SONAR_THRESHOLD = 40;     // Sonar threshold distance in centimeters

//******************************************************************************
// Global variables
//******************************************************************************
AF_MotorShield2 motorController;
AF_DCMotor2 leftMotor(motorController, 0);
AF_DCMotor2 rightMotor(motorController, 1);

Servo panServo;                     // For panning the ultrasonic sensor left and right
SonarSensor sonar(2, 4);            // Ultrasonic sensor on Arduino pins 2 and 4
IRProximitySensor proxRight(5, 0);  // Right IR Proximity sensor on pin 5 (for obstacle detection on the right side)
IRProximitySensor proxLeft(6, 0);   // Left IR Proximity sensor on pin 6 (for obstacle detection on the left side)
IRProximitySensor proxStep(7);      // Step IR Proximity sensor on pin 7 (for step, i.e. drop-off, detection)
IRSensor ir(3);                     // IR Remote Sensor on pin 3 (for receiving signals from IR remote)
IMU imu;                            // Inertial Measurement Unit (IMU) module

bool motorsEnabled = false;         // Indicates if the motors are enabled

//******************************************************************************
// TaskScheduler and Tasks
//******************************************************************************
TaskScheduler scheduler;
Task CheckRemoteTask(CheckRemoteCommand);
Task CheckMagCalibrationTask(CheckMagCalibration);
Task CheckStepSensorTask(CheckStepSensor);
Task CheckSideSensorsTask(CheckSideSensors);
Task CheckSonarSensorTask(CheckSonarSensor);
Task CheckCorneredTask(CheckCornered);
Task GoForwardTask([] () { GoForward(); return false; });


//******************************************************************************
// Arduino setup method - Performs initialization
//******************************************************************************
void setup()
{
    Serial.begin(57600);
    Wire.begin();
    pinMode(LED_PIN, OUTPUT);

    // Initialize AF Motor Shield
    motorController.Begin();

    // Initialize IMU
    if (!imu.Begin()) while (true) IndicateFailure(F("IMU failed to initialize."));

    // Configure Servo for Sonar sensor
    panServo.attach(SERVO_PIN);

    // Center servo to point straight ahead
    panServo.write(SERVO_BIAS);

    // Setup tasks
    scheduler.Add(CheckRemoteTask);
    scheduler.Add(CheckMagCalibrationTask);
    //scheduler.Add(CheckStepSensorTask);
    scheduler.Add(CheckSideSensorsTask);
    scheduler.Add(CheckSonarSensorTask);
    scheduler.Add(GoForwardTask);

    // Blink LED to indicate ready
    BlinkLEDCount(5, 250, 250);

    // Setup a 4 second watchdog timer to reset microcontroller if program locks up
    wdt_enable(WDTO_4S);
}


//******************************************************************************
// Arduino loop method - main program loop
//******************************************************************************
void loop()
{
    scheduler.Dispatch();
}


bool CheckRemoteCommand()
{
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


bool CheckMagCalibration()
{
    if (!motorsEnabled) return false;   // Motors have to be enabled to perform mag calibration

    PerformMagCalibration();
    motorsEnabled = false;
    scheduler.Remove(CheckMagCalibrationTask);

    return true;
}

bool CheckStepSensor()
{
    // Check if sensor triggered (triggered if sensor returns 0 (false))
    auto stepDetected = !proxStep.Read();

    if (!stepDetected || !motorsEnabled) return false;

    // If step sensor is triggered then stop and back up
    Logger() << F("Step sensor triggered") << endl;
    Stop();
    GoBackward();

    while (!proxStep.Read());   // Backup until step sensor turns off

    GoBackward(500);            // Then back up a little more
    Spin180();                  // And then turn around

    return true;
}


bool CheckSideSensors()
{
    // Read both left and right sensors
    auto left = proxLeft.Read();
    auto right = proxRight.Read();

    // If both sensors triggered then we may be in a corner or tight space
    if (left && right)
    {
        Logger() << F("Both left and right IR sensors triggered.") << endl;
        Stop();
        GoBackward();
        scheduler.InsertAt(CheckCorneredTask, 2);
    }
    else if (left)      // If left sensor triggered then spin right to avoid obstacle
    {
        Logger() << F("Left IR sensor triggered.") << endl;
        Spin('R');
    }
    else if (right)     // If right sensor triggered then spin left to avoid obstacle
    {
        Logger() << F("Right IR sensor triggered.") << endl;
        Spin('L');
    }
    else                // Neither sensor triggered
    {
        return false;
    }

    return true;
}


bool CheckCornered()
{
    Logger() << F("Backing out of corner.") << endl;
    GoBackward(500);                // Backup for 1/2 second more
    Spin180('R');                   // Then turn around
    scheduler.Remove(CheckCorneredTask);

    return true;
}


bool CheckSonarSensor()
{
    // Check for obstacle ahead
    auto ping = sonar.MultiPing();

    if (ping > SONAR_THRESHOLD) return false;
        
    // An obstacle was detected closer than threshold distance
    // Stop when obstacle is detected
    TRACE(Logger() << F("Obstacle detected ahead by sonar") << endl);
    Stop();

    // If obstacle is really close then back up a little first
    if (ping < SONAR_THRESHOLD / 2)
    {
        GoBackward(500);    // Backup for 1/2 second
    }

    // Try to find a better direction to move
    auto betterDirection = sonarScan();

    // Start a spin in the direction indicated by sonar
    Spin(betterDirection);

    auto timeout = millis() + 2000;

    // Keep turning until sonar no longer sees the obstacle
    for (auto ping = sonar.Ping(); ping != PING_FAILED && ping < SONAR_THRESHOLD; ping = sonar.Ping())
    {
        if (millis() > timeout) break;

        delay(50);
        wdt_reset();
    }

    // The obstacle is no longer detected, but it still may not be completely
    // clear of the robot. So continue turning a little longer to hopefully 
    // completely clear it. The time delay depends on the speed of the robot.
    delay(400);
    Stop();
    wdt_reset();

    return true;
}


//******************************************************************************
// Movement control methods.
//******************************************************************************

void Stop()
{
    SetMotors(0, 0);
}


void GoForward()
{
    SetMotors(CRUISE_SPEED, CRUISE_SPEED);
}


void GoBackward()
{
    SetMotors(-CRUISE_SPEED, -CRUISE_SPEED);
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
        SetMotors(-CRUISE_SPEED, CRUISE_SPEED);
    }
    else if (direction == 'L') // Spin to the left
    {
        SetMotors(CRUISE_SPEED, -CRUISE_SPEED);
    }
}


void Spin(char direction, uint32_t duration)
{
    auto timeout = millis() + duration;

    Spin(direction);

    while (millis() < timeout) wdt_reset();                 // Keep watchdog timer reset to prevent auto-restart

    Stop();
}


void Spin180(char direction)
{
    TRACE(Logger() << F("Spin180(") << direction << ')' << endl);

    // Start spin in requested direction
    Spin(direction);

    /*--------------------------------------------------------------------------
    The algoritm below measures the progress of the spin. It uses a simple
    integration of the gyro rate sensor in the IMU to determine how far we
    have turned over time. 
    
    The gyro reports angular rates about the sensors axes (i.e. the rate of turn
    in degrees per second) rather than inertial position angles. So, to determine
    an angle, we have to intergrate the angular rate over time:

         angle = sum((w0 + (w1 - w0) / 2.0) * dt) over intervals of dt

    This is the trapazoidal integration formula where w0 is the angular rate
    measurement from the previous iteraton (time t0), w1 is the angular rate
    measurement at the current iteration (time t1), and dt is the time interval
    between measurements (t1 - t0). It approximates the area under the angular
    rate curve over the interval dt as the sum of the area of a rectangle (w0 * dt),
    representing the contribution from the previous angular rate, and the area
    of small triangle (w1 -w0) * dt / 2, representing the contribution from the
    change in angular rate (w1 -w0) over the interval dt. The total angle traversed
    is the cumulative sum of these small areas from start to finish. To improve
    accuracy, dt should be very short (on the order of milliseconds).
    
    Since we are only interested in left/right turns of the robot (which are 
    rotations only in the robot's x-y plane) we can just use the z axis angular
    rate from the gyro (a rotation fixed in the x-y plane is a rotation about
    the z-axis).This assumes, of course, that the gyro sensor is mounted so that
    its x-y plane is parallel to the robot's x-y plane.

    It should be noted that a simple integration of the gyro rates is normally
    not recommended since the gyro rates drift over time, causing the measured
    angle to diverge from the true value. However, this caveat is usually made
    assuming that you are using the gyro to compute attitude angles that must
    remain stable and accurate over long periods. But we are just measuring how
    much we have turned so far, and a turn completes in just a few seconds. Over
    that short time the gyro drift is minimal and can be ignored, as long as you
    don't need to be super accurate (which we don't in this case).

    NOTE: All calculations are in radians!
    --------------------------------------------------------------------------*/
    float targetAngle = 180.0*DEG_TO_RAD;   // Target angle
    auto angle = 0.0f;                      // How much we have turned so far
    auto w0 = 0.0f;                         // Initial turn rate
    auto t0 = micros();                     // Time of last measurement

    // Use absolute value of current angle since we are only measuring the magnitude 
    // of the turn and not the direction
    for (auto t1 = t0; abs(angle) < targetAngle; t1 = micros())
    {
        wdt_reset();

        // Delay at least 4ms between measurements to ensure we have an updated measurement
        // Use UDIFF to compute difference of unsigned numbers (handles 32-bit wrap-around)
        if (UDIFF(t1, t0) < 4000) continue;

        // Get new gyro measurement and compute time since last measurement (in seconds)
        auto w1 = imu.GetGyro().z;
        auto dt = UDIFF(t1, t0) / 1000000.0f;

        // Update turn angle using trapezoidal integration
        angle += (w0 + (w1 - w0) / 2.0) * dt;

        // Update starting values for next iteration
        w0 = w1;
        t0 = t1;

        TRACE(Logger() << F("Spin180, ") << dt << _FLOAT(w0,2) << ',' << _FLOAT(w1,2) << ',' << _FLOAT(angle,2) << endl);
    }

    TRACE(Logger() << F("Spin180: turn complete") << endl);
    Stop();
}


void PerformMagCalibration()
{
    /*--------------------------------------------------------------------------
     This routine performs a real-time calibration of the IMU magnetometer x 
     and y axis bias offsets. The basic idea is to take x and y magnetometer
     readings while the robot is spinning in a circle, and note the minimum and
     maximum values on each axis. The bias offset for each axis is the average 
     of these minimum and maximum values.
     
     The magnetometer values can be a bit noisy and are suseptible to producing
     occasional outliers. To ensure that these outliers do not unduly affect
     the calibration, the magnetomer values are smoothed with a low-pass filter
     before updating the min/max values.
    
     NOTE: This process only calibrates the bias offset for the magnetometer
           x and y axes. This is sufficient for devices that operate primarily
           in the x-y plane. If the mag z-axis value is needed then it must be 
           calibrated separately.
    --------------------------------------------------------------------------*/
    TRACE(Logger() << F("MagCalibration.Begin") << endl);

    auto xmin =  1000.0f;
    auto xmax = -1000.0f;
    auto ymin =  1000.0f;
    auto ymax = -1000.0f;
    auto alpha = 0.1f;
    auto beta = 1 - alpha;
    auto mag = imu.GetMagRaw();

    for (auto endtime = millis() + 100; millis() < endtime;  mag = imu.GetMagRaw()) delay(10);

    auto x = mag.x;
    auto y = mag.y;

    Spin('R');
    delay(20);

    auto t0 = millis();
    auto endTime = t0 + 6000;

    for (auto t1 = t0; t1 < endTime; t1 = millis())
    {
        wdt_reset();

        if (t1 < (t0+10)) continue;

        mag = imu.GetMagRaw();
        x = beta * x + alpha * mag.x;
        y = beta * y + alpha * mag.y;
        xmin = min(x, xmin);
        xmax = max(x, xmax);
        ymin = min(y, ymin);
        ymax = max(y, ymax);

        TRACE(Logger() << F("MagCalibration.Phase1, ") << mag.x << ',' << mag.y << ',' << x << ',' << y << ','
                       << _FLOAT(xmin, 2) << ',' << _FLOAT(xmax, 2) << ',' << _FLOAT(ymin, 2) << ',' << _FLOAT(ymax, 2)
                       << endl);
    }

    Stop();
    delay(20);
    Spin('L');
    delay(20);

    t0 = millis();
    endTime = t0 + 5000;

    for (auto t1 = t0; t1 < endTime; t1 = millis())
    {
        wdt_reset();

        if (t1 < (t0 + 10)) continue;

        mag = imu.GetMagRaw();
        x = beta * x + alpha * mag.x;
        y = beta * y + alpha * mag.y;
        xmin = min(x, xmin);
        xmax = max(x, xmax);
        ymin = min(y, ymin);
        ymax = max(y, ymax);

        TRACE(Logger() << F("MagCalibration.Phase2, ") << mag.x << ',' << mag.y << ',' << x << ',' << y << ','
                       << _FLOAT(xmin, 2) << ',' << _FLOAT(xmax, 2) << ',' << _FLOAT(ymin, 2) << ',' << _FLOAT(ymax, 2)
                       << endl);
    }

    Stop();

    auto xBias = (xmin + xmax) / 2.0f;
    auto yBias = (ymin + ymax) / 2.0f;

    imu.SetMagBias(xBias, yBias);

    TRACE(Logger() << F("MagCalibration.Complete, xBias=") << xBias << F(", yBias=") << yBias << endl);
}


//******************************************************************************
// Set the motor speed
// The speed of both motors is constrained to the range -255 to +255.
//******************************************************************************
void SetMotors(int leftSpeed, int rightSpeed)
{
    if (!motorsEnabled) leftSpeed = rightSpeed = 0;
  
    leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);
  
    // If the motors don't rotate in the desired direction you can correct it
    // by changing the positive/negative sign of the speed in these calls.
    leftMotor.Run(leftSpeed);
    rightMotor.Run(rightSpeed);
}


//******************************************************************************
// Do one ultrasonic sensor ping.
//******************************************************************************
uint16_t Ping()
{
    uint16_t ping;

    // Keep retrying until we get a good ping (or the watchdog timer resets the board)
    while ((ping = sonar.PingCentimeters()) == PING_FAILED) delay(40);

    return ping;
}


//******************************************************************************
// Use the ultrasonic sensor to scan from right to left. 
// Return whichever direction has more open space (i.e., the direction in which 
// the ultrasonic sensor pings the greatest distances). Return value is 'L' if
// the left side appears more open, or 'R' if the right side appears more open.
//******************************************************************************
char sonarScan()
{
    static const int SCAN_ANGLE = 60;
    
    uint16_t leftscan[SCAN_ANGLE];
    uint16_t rightscan[SCAN_ANGLE];

    panServo.write(SERVO_BIAS);  // Start sonar at center position looking straight ahead
    delay(100);                 // Wait 100ms for servo to reach start position
    wdt_reset();

    ZERO_ARRAY(leftscan);
    ZERO_ARRAY(rightscan);

    // Scan distances from center to max right angle (right side cw)
    for (auto i = 1; i <= SCAN_ANGLE; i++)
    {
        panServo.write(SERVO_BIAS - i); 
        delay(2);				// Delay 2ms for servo to move to next position
        rightscan[i-1] += Ping();
        wdt_reset();
    }

    // Scan distances from max right angle to center (right side ccw)
    for (auto i = SCAN_ANGLE; i >= 1; i--)
    {
        panServo.write(SERVO_BIAS - i); 
        delay(2);				// Delay 2ms for servo to move to next position
        rightscan[i-1] += Ping();
        wdt_reset();
    }

    // Scan distances from center to max left (left side ccw)
    for (auto i = 1; i <= SCAN_ANGLE; i++)
    {
        panServo.write(SERVO_BIAS + i); 
        delay(2);				// Delay 2ms for servo to move to next position
        leftscan[i-1] += Ping();
        wdt_reset();
    }

    // Scan the distance from max left angle to center (left side cw)
    for (auto i = SCAN_ANGLE; i >= 1; i--)
    {
        panServo.write(SERVO_BIAS + i); 
        delay(2);				// Delay 2ms for servo to move to next position
        leftscan[i-1] += Ping();
        wdt_reset();
    }

    // All done scanning - recenter servo to point straight ahead again
    panServo.write(SERVO_BIAS);
  
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
        digitalWrite(LED_PIN, HIGH);
        delay(onTime);
        digitalWrite(LED_PIN, LOW);
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

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

#include "Robot_7_Tank.h"


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
Task GoForwardTask([]() { GoForward(); return false; });
Task AvoidObstacleDetectedBySonarTask(AvoidObstacleDetectedBySonar);


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
    scheduler.Add(CheckStepSensorTask);
    scheduler.Add(CheckSideSensorsTask);
    scheduler.Add(CheckSonarSensorTask);
    scheduler.Add(GoForwardTask);

    // Blink LED to indicate ready
    BlinkLEDCount(5, 150, 250);

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


//******************************************************************************
// Task methods
//******************************************************************************

#define REMOTE_DEBOUNCE_TIME 500

// The timeout helps to eliminate rapid, repeated triggers (bounce) caused by pulsed signals from IR Remote
static uint32_t remoteDebounceTimeout = 0;

bool CheckRemoteCommand()
{
    auto now = millis();

    // Check if the IR Remote Sensor detected a signal
    if (!ir.Read() || (now < remoteDebounceTimeout))
    {
        if (now > (remoteDebounceTimeout - (REMOTE_DEBOUNCE_TIME/2))) digitalWrite(LED_PIN, LOW);

        return false;
    }

    TRACE(Logger() << F("Recieved IR Remote signal") << endl);
    digitalWrite(LED_PIN, HIGH);
    Stop();     // If motors are running this ensures they are stopped before being disabled, otherwise it does nothing
    motorsEnabled = !motorsEnabled;
    remoteDebounceTimeout = millis() + REMOTE_DEBOUNCE_TIME;

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
    TRACE(Logger() << F("Step sensor triggered") << endl);
    Stop();
    GoBackward();

    while (!proxStep.Read());   // Backup until step sensor turns on again

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
        TRACE(Logger() << F("Both left and right IR sensors triggered.") << endl);
        Stop();
        GoBackward(500);    // Backup for 1/2 second
        Spin180();          // Then turn around
    }
    else if (left)          // If left sensor triggered then spin right to avoid obstacle
    {
        TRACE(Logger() << F("Left IR sensor triggered.") << endl);
        Spin('R');
    }
    else if (right)         // If right sensor triggered then spin left to avoid obstacle
    {
        TRACE(Logger() << F("Right IR sensor triggered.") << endl);
        Spin('L');
    }
    else                    // Neither sensor triggered
    {
        return false;
    }

    return true;
}


static char   scanDirection = 'R';
static int8_t scanAngle = 0;

bool CheckSonarSensor()
{
    if (!motorsEnabled) return false;

    // Check for obstacle ahead
    auto ping = sonar.MultiPing();

    if (ping > SONAR_THRESHOLD)
    {
        // Pan the sonar back and forth +/-10 degrees.
        if (scanDirection == 'R')
        {
            if (++scanAngle > 10) scanDirection = 'L';
        }
        else
        {
            if (--scanAngle < -10) scanDirection = 'R';
        }

        panServo.write(SERVO_BIAS + scanAngle);

        return false;
    }

    // An obstacle was detected closer than threshold distance
    // Stop when obstacle is detected
    TRACE(Logger() << F("Obstacle detected ahead by sonar") << endl);
    Stop();
    panServo.write(SERVO_BIAS);
    scanAngle = 0;

    // If obstacle is really close then back up a little first
    if (Ping() < SONAR_THRESHOLD / 2)
    {
        GoBackward(500);    // Backup for 1/2 second
    }

    // Add task to scan for a better direction to move
    scheduler.InsertAfter(AvoidObstacleDetectedBySonarTask, CheckRemoteTask);

    return true;
}


bool AvoidObstacleDetectedBySonar()
{
    // Try to find a better direction to move
    auto results = ScanForBetterDirection();

    if (results.RightMax < SONAR_THRESHOLD && results.LeftMax < SONAR_THRESHOLD)
    {
        // We are stuck in a tight space. Best to backup, turn around, and get out.
        TRACE(Logger() << F("Backing out of space.") << endl);
        GoBackward(500);                // Backup for 1/2 second more
        Spin180(results.BestDirection); // Then turn around
    }
    else
    {
        // Start a spin in the direction indicated by sonar
        Spin(results.BestDirection);

        auto timeout = millis() + 4000;

        // Keep turning until sonar no longer sees the obstacle (or timeout)
        while (Ping() < SONAR_THRESHOLD)
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
    }

    // Remove task as it is no longer needed (for now)
    scheduler.Remove(AvoidObstacleDetectedBySonarTask);

    return true;
}

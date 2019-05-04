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

#include <RTL_I2C.h>
#include <RTL_Variant.h>
#include <RTL_Stdlib.h>
#include <RTL_Queue.h>
#include <RTL_Math.h>
#include <RTL_List.h>
#include <RTL_Debug.h>
#include <RTL_Conversions.h>

#define DEBUG 1

#include "Robot_7_Tank.h"


//******************************************************************************
// Global variables
//******************************************************************************
AF_MotorShield2 motorController;
AF_DCMotor2 rightMotor(motorController, 0);
AF_DCMotor2 leftMotor(motorController, 1);

Servo panServo;                     // For panning the ultrasonic sensor left and right
SonarSensor sonar(2, 4);            // Ultrasonic sensor on Arduino pins 2 and 4
IRProximitySensor proxRight(5, 0);  // Right IR Proximity sensor on pin 5 (for obstacle detection on the right side)
IRProximitySensor proxFront(6);     // Front IR Proximity sensor on pin 12 (for obstacle detection ahead)
IRProximitySensor proxLeft(7, 0);   // Left IR Proximity sensor on pin 6 (for obstacle detection on the left side)
IRProximitySensor proxStep(12);     // Step IR Proximity sensor on pin 7 (for step, i.e. drop-off, detection)
IMU imu;                            // Inertial Measurement Unit (IMU) module
Blinker heartbeat(1000,5);          // 50ms flash every second (50ms = 5% of 1000) 

StatusReg status;

//******************************************************************************
// TaskScheduler and Tasks
//******************************************************************************
TaskScheduler scheduler;
Task CheckRemoteTask(CheckRemoteCommand);
Task CheckMagCalibrationTask(CheckMagCalibration);
Task CheckStepSensorTask(CheckStepSensor);
Task CheckFrontSensorsTask(CheckFrontSensor);
Task CheckSideSensorsTask(CheckSideSensors);
Task CheckSonarSensorTask(CheckSonarSensor);
Task ScanForNewDirectionTask(ScanForNewDirection);
//Task GoTask([]() { Go(); return false; });


//******************************************************************************
// Arduino setup method - Performs initialization
//******************************************************************************
void setup()
{
    int STEP_INIT_I2C_SHIELD;
    int STEP_INIT_IMU;
    int STEP_INIT_IRREMOTE;
    int STEP_INIT_MOTORCTRLR;
    int STEP_BEGIN_IMU;
    int STEP_BEGIN_MOTOR_CTLR;

    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);

    I2c.begin();
    I2c.setSpeed(I2C::StdSpeed);
    I2c.pullup(I2C::DisablePullup);

    //I2c.scan();

    //--------------------------------------------------------------------------
    // Determine if motor controller exists, configure it, and ensure motors are
    // stopped. This is done first to ensure the robot is in a stopped state in 
    // case the watchdog timer reset the CPU while the motors were running.
    // Note: The motor controller is a shield connected directly to the Arduino,
    //       so it does not depend on the I2C shield.
    //--------------------------------------------------------------------------
    //BlinkLEDCount(STEP_INIT_MOTORCTRLR = 1, 50, 200);
    status.MOTOR_CTLR_VALID = I2c.detect(MOTOR_SHIELD_I2C_ADDRESS, F("Motor shield"));

    if (!status.MOTOR_CTLR_VALID) IndicateFailure(STEP_INIT_MOTORCTRLR, F("Motor shield not found."), true);

    //--------------------------------------------------------------------------
    // Configure Motor shield
    //--------------------------------------------------------------------------
    //delay(1000);
    //BlinkLEDCount(STEP_BEGIN_MOTOR_CTLR = 6, 50, 200);
    motorController.Begin();
    Logger() << F("Motor shield configured.") << endl;
    Stop();     // Ensure the motors are stopped

    //--------------------------------------------------------------------------
    // Determine if I2C shield exists
    // If I2C shield not found then assume I2C devices are connected directly to 
    // the Arduino I2C bus
    //--------------------------------------------------------------------------
    BlinkLEDCount(STEP_INIT_I2C_SHIELD=2, 50, 200);
    status.I2C_SHILED_VALID = I2c.detect(I2C_SHIELD_I2C_ADDRESS, F("I2C Shield"));

    // If found, must configure I2C shield before continuing initialization since other I2C devices may be connected via this shield
    if (status.I2C_SHILED_VALID)
    {
        // Enable ports 0 & 1 on the I2C shield - 0=IR remote, 1=IMU
        I2c.write(I2C_SHIELD_I2C_ADDRESS, I2C_SHIELD_PORT0 | I2C_SHIELD_PORT1);
        //I2C_SendCommand(I2C_SHIELD_I2C_ADDRESS, I2C_SHIELD_PORT0 | I2C_SHIELD_PORT1);
    }

    //--------------------------------------------------------------------------
    // Determine connectivity to IMU
    //--------------------------------------------------------------------------
    delay(1000);
    BlinkLEDCount(STEP_INIT_IMU=3, 50, 200);
    status.IMU_VALID = I2c.detect(RAZOR_IMU_ADDRESS,  F("IMU"), 20);
    
    //--------------------------------------------------------------------------
    // Determine connectivity to IR Remote receiver
    //--------------------------------------------------------------------------
    delay(1000);
    BlinkLEDCount(STEP_INIT_IRREMOTE=4, 50, 200);
    status.IR_REMOTE_VALID = I2c.detect(IR_REMOTE_I2C_ADDRESS, F("IR remote receiver"));
    
    //--------------------------------------------------------------------------
    // If IR remote receiver or motor controller not found then abort since we can't do anything)
    //--------------------------------------------------------------------------
    if (!status.IR_REMOTE_VALID) IndicateFailure(STEP_INIT_IRREMOTE, F("IR Remote receiver not found."), true);

    //--------------------------------------------------------------------------
    // If IMU not found or fails to initialize then use some other means for attitude control (e.g., timing)
    //--------------------------------------------------------------------------
    //status.IMU_VALID = false; // temporary

    if (status.IMU_VALID)
    {
        delay(1000);
        BlinkLEDCount(STEP_BEGIN_IMU = 5, 50, 200);

        if (imu.Begin())
        {
            Logger() << F("IMU configured.") << endl;
        }
        else
        {
            Logger() << F("IMU failed to initialize.") << endl;
            status.IMU_VALID = false;
        }
    }

    //--------------------------------------------------------------------------
    // Configure Servo for Sonar sensor
    //--------------------------------------------------------------------------
    panServo.attach(SERVO_PIN);
    PanSonar(-90);      // Pan sonar through full range
    delay(1000);
    PanSonar(90);
    delay(1000);
    PanSonar(0);        // Center servo to point straight ahead
    Logger() << F("Sonar servos configured.") << endl;

    //--------------------------------------------------------------------------
    // Setup tasks (tasks are executed in the ordered they are added)
    //--------------------------------------------------------------------------
    scheduler.Add(CheckRemoteTask);
    //scheduler.Add(CheckMagCalibrationTask);
    scheduler.Add(CheckStepSensorTask);
    scheduler.Add(CheckFrontSensorsTask);
    scheduler.Add(CheckSideSensorsTask);
    scheduler.Add(CheckSonarSensorTask);
    //scheduler.Add(GoTask);
    Logger() << F("Tasks have been scheduled.") << endl;

    //--------------------------------------------------------------------------
    // Start LED heartbeat to indicate heathly status;
    // Ensure the robot begins in the stopped state;
    // Setup a 4 second watchdog timer to reset microcontroller if program locks up
    //--------------------------------------------------------------------------
    heartbeat.Start();
    wdt_enable(WDTO_4S);

    Logger() << F("Robot ready.") << endl << endl;
}


//******************************************************************************
// Arduino loop method - main program loop
//******************************************************************************
void loop()
{
    wdt_reset();
    scheduler.Dispatch();
    heartbeat.Poll();
}


//******************************************************************************
// Task methods
//******************************************************************************

bool CheckRemoteCommand()
{
    static uint32_t lastCommandTime = 0;
    static uint32_t lastCommandCode = IR_NONE;

    uint32_t now = millis();
    IRRemoteCommand response;

    I2c.read(IR_REMOTE_I2C_ADDRESS, response);

    if (response.Code != IR_NONE)
    {
        lastCommandCode = response.Code;
        lastCommandTime = now;
    }
    else if (lastCommandCode != IR_NONE && (now - lastCommandTime) > 200)
    {
        response.Type = IRRemoteCommandType::End;
        response.Code = lastCommandCode;
        lastCommandCode = IR_NONE;
    }
    else
    {
        return false;
    }

    return ProcessCommand(response);
}


bool CheckMagCalibration()
{
    if (!isMoving) return false;   // Have to start moving before performing mag calibration

    PerformMagCalibration();
    scheduler.Remove(CheckMagCalibrationTask);

    return true;
}


bool CheckStepSensor()
{
    // Check if sensor triggered (triggered if sensor returns 0 (false))
    auto stepDetected = !proxStep.Read();

    if (!isMoving || !stepDetected) return false;

    // If step sensor is triggered then stop and back up
    Logger() << F("Step sensor triggered") << endl;
    Stop();
    GoBackward();

    while (!proxStep.Read());   // Backup until step sensor turns on again

    // Keep backing up a little longer to be sure we have backed far enough away for the edge
    delay(250);

    // Determine the best way to turn around
    char spinDirection;

    do
    {
        spinDirection = !proxLeft.Read() ? 'L' : (!proxRight.Read() ? 'R' : 'B'); // 'B' means backup some more
        wdt_reset();
    }
    while (spinDirection == 'B');

    Spin180(spinDirection);     // Turn around
    GoForward();                // And start moving forward again

    return true;
}


bool CheckFrontSensor()
{
    // Check if sensor triggered
    if (status.INHIBIT_SENSORS || !isMoving || !proxFront.Read()) return false;

    // If front sensor is triggered then stop and back up
    Logger() << F("Front sensor triggered") << endl;
    Stop();
    GoBackward();

    while (proxFront.Read());   // Backup until sensor turns off again

    wdt_reset();
    GoBackward(500);            // Then back up a little more

    // Add task to scan for a better direction to move
    scheduler.InsertAfter(ScanForNewDirectionTask, CheckRemoteTask);

    return true;
}


bool CheckSideSensors()
{
    if (status.INHIBIT_SENSORS || !isMoving) return false;

    // Read both left and right sensors
    auto left = proxLeft.Read();
    auto right = proxRight.Read();

    if (left && right)      // If both sensors triggered then we may be in a corner or tight space
    {
        Logger() << F("Both left and right IR sensors triggered.") << endl;
        Stop();
        GoBackward(500);            // Backup for 1/2 second
        Spin180();                  // Then turn around
        GoForward();
    }
    else if (left)          // If left sensor triggered then spin right to avoid obstacle
    {
        Logger() << F("Left IR sensor triggered.") << endl;
        Spin('R');                  // Start a turn away from the obstacle

        while (proxLeft.Read());    // Turn until sensor stops detecting

        wdt_reset();
        delay(200);                 // Turn a little longer to be sure
        Go();
    }
    else if (right)         // If right sensor triggered then spin left to avoid obstacle
    {
        Logger() << F("Right IR sensor triggered.") << endl;
        Spin('L');                  // Start a turn away from the obstacle

        while (proxRight.Read());   // Turn until sensor stops detecting

        wdt_reset();
        delay(200);                 // Turn a little longer to be sure
        Go();
    }
    else                    // Neither sensor triggered
    {
        return false;
    }

    return true;
}


static char    scanDirection = 'R';
static int8_t  scanAngle = 0;
static int16_t t1PingCount = 0;
static int16_t t2PingCount = 0;


bool CheckSonarSensor()
{
    if (status.INHIBIT_SENSORS || !isMoving) return false;

    // Pan the sonar back and forth +/-10 degrees.
    PanSonar(scanAngle);

    if (scanDirection == 'R')
    {
        if (++scanAngle > 10) scanDirection = 'L';
    }
    else
    {
        if (--scanAngle < -10) scanDirection = 'R';
    }

    // Check for obstacle ahead
    auto ping = sonar.MultiPing();

    if (ping > SONAR_THRESHOLD)
    {
        t2PingCount = 0;

        // Resume normal speed if we got 10 long pings in a row
        if (goingSlow && t1PingCount++ > 10) GoForward();

        return false;
    }

    // An obstacle was detected closer than threshold distance
    Logger() << F("Obstacle detected ahead by sonar") << endl;

    // if obstacle is greater than minimum distance then just slow down
    if (ping > SONAR_THRESHOLD2)
    {
        t1PingCount = 0;

        // Slow down if we got 5 short pings in a row
        if (!goingSlow && t2PingCount++ > 5) GoSlow();

        return false;
    }

    // Stop when obstacle is closer than minimum distance
    Stop();
    PanSonar(0);
    scanAngle = 0;

    // If obstacle is really close then back up a little first
    if (ping <= SONAR_THRESHOLD3)
    {
        GoBackward(500);    // Backup for 1/2 second
    }

    // Add task to scan for a better direction to move
    scheduler.InsertAfter(ScanForNewDirectionTask, CheckRemoteTask);

    //ScanForNewDirection();

    return true;
}


bool ScanForNewDirection()
{
    // Try to find a better direction to move
    auto results = ScanForBetterDirection();

    if (results.RightMax < SONAR_THRESHOLD && results.LeftMax < SONAR_THRESHOLD)
    {
        // We are stuck in a tight space. Best to backup, turn around, and get out.
        TRACE(Logger() << F("Backing out of space.") << endl);
        GoBackward(500);                // Backup for 1/2 second more
        Spin180(results.BestDirection); // Then turn around
        GoForward();
        wdt_reset();
    }
    else
    {
        TRACE(Logger() << F("Sonar: best direction is ") << results.BestDirection << endl);

        // Start a spin in the direction indicated by sonar
        Spin(results.BestDirection);

        auto timeout = millis() + 1000;

        // Keep turning until sonar no longer sees the obstacle (or timeout)
        while (Ping() < SONAR_THRESHOLD && millis() < timeout)
        {
            delay(50);
            wdt_reset();
        }

        // The obstacle is no longer detected, but it still may not be completely
        // clear of the robot. So continue turning a little longer to hopefully 
        // completely clear it.
        delay(200);
        GoForward();
        wdt_reset();
    }

    // Remove task as it is no longer needed (for now)
    scheduler.Remove(ScanForNewDirectionTask);

    return true;
}


bool ProcessCommand(IRRemoteCommand& command)
{
    TRACE(Logger() << F("Recieved IR Remote command: ") << _HEX(command.Code) << F(", type=") << _HEX(command.Type) << endl);

    auto didSomething = false;

    switch (command.Code)
    {
        case IR_PLAY:         // Start / Stop
            // Ignore repeat and end commands for PLAY button
            if (command.Type == IRRemoteCommandType::Normal)
            {
                if (isMoving) Stop(); else GoForward();

                TRACE(Logger() << (isMoving ? F("Started") : F("Stopped")) << endl);
                didSomething = true;
            }
            break;

        case IR_PREV:         // Turn left
            if (command.Type == IRRemoteCommandType::Normal)
            {
                TRACE(Logger() << F("Turning left") << endl);
                status.INHIBIT_SENSORS = true;
                Spin('L');
                didSomething = true;
            }
            else if (command.Type == IRRemoteCommandType::End)
            {
                TRACE(Logger() << F("Cancelling turn") << endl);
                status.INHIBIT_SENSORS = false;
                if (isMoving) Go(); else Stop();
            }
            break;

        case IR_NEXT:         // Turn right
            if (command.Type == IRRemoteCommandType::Normal)
            {
                TRACE(Logger() << F("Turning right") << endl);
                status.INHIBIT_SENSORS = true;
                Spin('R');
                didSomething = true;
            }
            else if (command.Type == IRRemoteCommandType::End)
            {
                TRACE(Logger() << F("Cancelling turn") << endl);
                status.INHIBIT_SENSORS = false;
                if (isMoving) Go(); else Stop();
            }
            break;

        case IR_VOL_PLUS:     // TODO: Increase speed
            break;

        case IR_VOL_MINUS:    // TODO: Reduce speed
            break;

        case IR_VOL_EQ:       // TODO: Resume normal forward speed
            break;

        case IR_CH:           // Backup
            if (command.Type == IRRemoteCommandType::Normal)
            {
                TRACE(Logger() << F("Backing up") << endl);
                status.INHIBIT_SENSORS = true;
                GoBackward();
                didSomething = true;
            }
            else if (command.Type == IRRemoteCommandType::End)
            {
                TRACE(Logger() << F("Cancelling back up") << endl);
                status.INHIBIT_SENSORS = false;
                Stop();
            }
            break;

        default:
            break;
    }

    return didSomething;
}

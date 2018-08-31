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
IRProximitySensor proxFront(12);    // Front IR Proximity sensor on pin 12 (for obstacle detection ahead)
IRProximitySensor proxRight(5, 0);  // Right IR Proximity sensor on pin 5 (for obstacle detection on the right side)
IRProximitySensor proxLeft(6, 0);   // Left IR Proximity sensor on pin 6 (for obstacle detection on the left side)
IRProximitySensor proxStep(7);      // Step IR Proximity sensor on pin 7 (for step, i.e. drop-off, detection)
IMU imu;                            // Inertial Measurement Unit (IMU) module

bool inhibitSensors = false;        // Indicates if sensors should be disabled

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
    Serial.begin(115200);
    Wire.begin();
    pinMode(LED_PIN, OUTPUT);

    // Enable ports 0 & 1 on the I2C shield - 0=IR remote, 1=IMU
    I2C_SendCommand(I2C_SHIELD_I2C_ADDRESS, I2C_SHIELD_PORT0 | I2C_SHIELD_PORT1);

    // Initialize AF Motor Shield
    motorController.Begin();

    // Initialize IMU
    if (!imu.Begin()) while (true) IndicateFailure(F("IMU failed to initialize."));

    // Configure Servo for Sonar sensor
    panServo.attach(SERVO_PIN);

    // Center servo to point straight ahead
    PanSonar(0);

    // Setup tasks
    scheduler.Add(CheckRemoteTask);
    //scheduler.Add(CheckMagCalibrationTask);
    scheduler.Add(CheckStepSensorTask);
    scheduler.Add(CheckFrontSensorsTask);
    scheduler.Add(CheckSideSensorsTask);
    scheduler.Add(CheckSonarSensorTask);
    //scheduler.Add(GoTask);

    Stop();   // Ensure the robot starts from the stopped state
    
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

bool CheckRemoteCommand()
{
    static uint32_t lastCommandTime = 0;
    static uint32_t lastCommandCode = IR_NONE;

    uint32_t now = millis();
    IRRemoteCommand command;
    
    I2C_Request(IR_REMOTE_I2C_ADDRESS, command);

    if (command.Code != IR_NONE)
    {
        lastCommandCode = command.Code;
        lastCommandTime = now;
    }
    else if (lastCommandCode != IR_NONE && (now - lastCommandTime) > 200)
    {
        command.Type = IRRemoteCommandType::End;
        command.Code = lastCommandCode;
        lastCommandCode = IR_NONE;
    }
    else
    {
        return false;
    }

    return ProcessCommand(command);
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
    TRACE(Logger() << F("Step sensor triggered") << endl);
    Stop();
    GoBackward();

    while (!proxStep.Read());   // Backup until step sensor turns on again

    GoBackward(500);            // Then back up a little more
    Spin180();                  // Turn around
    GoForward();                // And start moving forward again

    return true;
}


bool CheckFrontSensor()
{
    // Check if sensor triggered
    if (inhibitSensors || !isMoving || !proxFront.Read()) return false;

    // If front sensor is triggered then stop and back up
    TRACE(Logger() << F("Front sensor triggered") << endl);
    Stop();
    GoBackward();

    while (proxFront.Read());   // Backup until step sensor turns on again

    wdt_reset();
    GoBackward(500);            // Then back up a little more

    // Add task to scan for a better direction to move
    scheduler.InsertAfter(ScanForNewDirectionTask, CheckRemoteTask);

    return true;
}


bool CheckSideSensors()
{
    if (inhibitSensors || !isMoving) return false;

    // Read both left and right sensors
    auto left = proxLeft.Read();
    auto right = proxRight.Read();

    if (left && right)      // If both sensors triggered then we may be in a corner or tight space
    {
        TRACE(Logger() << F("Both left and right IR sensors triggered.") << endl);
        Stop();
        GoBackward(500);            // Backup for 1/2 second
        Spin180();                  // Then turn around
        GoForward();
    }
    else if (left)          // If left sensor triggered then spin right to avoid obstacle
    {
        TRACE(Logger() << F("Left IR sensor triggered.") << endl);
        Spin('R');                  // Start a turn away from the obstacle

        while (proxLeft.Read());    // Turn until sensor stops detecting
        
        wdt_reset();
        delay(200);                 // Turn a little longer to be sure
        Go();
    }
    else if (right)         // If right sensor triggered then spin left to avoid obstacle
    {
        TRACE(Logger() << F("Right IR sensor triggered.") << endl);
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
    if (inhibitSensors || !isMoving) return false;

    // Check for obstacle ahead
    PanSonar(scanAngle);

    auto ping = sonar.MultiPing();
        
    // Pan the sonar back and forth +/-10 degrees.
    if (scanDirection == 'R')
    {
        if (++scanAngle > 10) scanDirection = 'L';
    }
    else
    {
        if (--scanAngle < -10) scanDirection = 'R';
    }

    if (ping > SONAR_THRESHOLD)
    {
        t2PingCount = 0;

        // Resume normal speed if we got 10 long pings in a row
        if (goingSlow && t1PingCount++ > 10) GoForward();

        return false;
    }

    // An obstacle was detected closer than threshold distance
    TRACE(Logger() << F("Obstacle detected ahead by sonar") << endl);
    
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
        // Start a spin in the direction indicated by sonar
        Spin(results.BestDirection);

        auto timeout = millis() + 2000;

        // Keep turning until sonar no longer sees the obstacle (or timeout)
        while (Ping() < SONAR_THRESHOLD)
        {
            if (millis() > timeout) break;

            delay(50);
            wdt_reset();
        }

        // The obstacle is no longer detected, but it still may not be completely
        // clear of the robot. So continue turning a little longer to hopefully 
        // completely clear it.
        delay(400);
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

              TRACE(Logger() << (isMoving ? F("Started") : F("Stopped"))  << endl);
              didSomething = true;
          }
      break;
      
      case IR_PREV:         // Turn left
          if (command.Type == IRRemoteCommandType::Normal)
          {
              TRACE(Logger() << F("Turning left") << endl);
              inhibitSensors = true;
              Spin('L');
              didSomething = true;
          }
          else if (command.Type == IRRemoteCommandType::End)
          {
              TRACE(Logger() << F("Cancelling turn") << endl);
              inhibitSensors = false;
              if (isMoving) Go(); else Stop();
          }
      break;
      
      case IR_NEXT:         // Turn right
          if (command.Type == IRRemoteCommandType::Normal)
          {
              TRACE(Logger() << F("Turning right") << endl);
              inhibitSensors = true;
              Spin('R');
              didSomething = true;
          }
          else if (command.Type == IRRemoteCommandType::End)
          {
              TRACE(Logger() << F("Cancelling turn") << endl);
              inhibitSensors = false;
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
              inhibitSensors = true;
              GoBackward();
              didSomething = true;
          }
          else if(command.Type == IRRemoteCommandType::End)
          {
              TRACE(Logger() << F("Cancelling back up") << endl);
              inhibitSensors = false;
              Stop();
          }
          break;

      default:
      break;
    }

    return didSomething;
}


#ifndef _Robot_7_Tank_h_
#define _Robot_7_Tank_h_

#include <Arduino.h>
#include <avr/wdt.h>
#include <Servo.h>
#include <Wire.h>
#include <RTL_Stdlib.h>
#include <RTL_I2C.h>
#include <RTL_TaskScheduler.h>
#include <AF_MotorShield2.h>
#include <AF_DCMotor2.h>
#include <IRSensor.h>
#include <IRProximitySensor.h>
#include <SonarSensor.h>
#include "IMU.h"
#include "RTL_IR_RemoteDecoder\RTL_IR_RemoteDecoder.h"


#define I2C_SHIELD_I2C_ADDRESS ((byte)0x77)
#define I2C_SHIELD_PORT0       ((byte)0b00000001)
#define I2C_SHIELD_PORT1       ((byte)0b00000010)
#define I2C_SHIELD_PORT2       ((byte)0b00000100)
#define I2C_SHIELD_PORT3       ((byte)0b00001000)


//******************************************************************************
// Sonar scan results
//******************************************************************************
struct SonarScanResults
{
    char BestDirection;     // Either 'L' (left) or 'R' (right)
    float LeftSum;          // Sum of left side
    float RightSum;         // Sum of right side
    float LeftMax;          // Max on left side
    float RightMax;         // Max on right side

    //int16_t MinAngle;
    //int16_t MaxAngle;
    //byte    CountZones = sizeof(ZoneSums) / sizeof(ZoneSums[0]);
    //float   ZoneSums[4];
    //int16_t BestAngle;

    //public: SonarScanResults() {};

    //public: SonarScanResults(int16_t _minAngle, int16_t _maxAngle)
    //{
    //    MinAngle = _minAngle;
    //    MaxAngle = _maxAngle;
    //    for (auto i = 0; i < CountZones; i++) ZoneSums[i] = 0.0f;
    //}
};


//******************************************************************************
// Forward declarations
//******************************************************************************
bool CheckRemoteCommand();
bool CheckStepSensor();
bool CheckFrontSensor();
bool CheckSideSensors();
bool CheckSonarSensor();
bool CheckMagCalibration();
bool ScanForNewDirection();

void Stop();
void Go();
void Go(int speed);
void GoSlow();
void GoForward();
void GoBackward();
void GoBackward(uint32_t duration);
void GoBackward(uint32_t duration, bool(*predicate)());
void Spin(int16_t angle);
void Spin(char direction);
void Spin(char direction, uint32_t duration);
void Spin180(char direction = 'R');
void SetMotors(int leftSpeed, int rightSpeed);

uint16_t Ping();
SonarScanResults ScanForBetterDirection();
void PanSonar(int angle);

void PerformMagCalibration();

void BlinkLEDCount(uint16_t count, uint16_t onTime = 100, uint16_t offTime = 100);
void IndicateFailure(const __FlashStringHelper* msg = nullptr);


//******************************************************************************
// Constants
//******************************************************************************
const int LED_PIN = 13;             // Hardware LED pin
const int SERVO_PIN = 9;            // Servo on Arduino pin 9
const int MAX_SPEED = 255;          // Maximum motor speed
const int CRUISE_SPEED = 180;       // Normal running speed
const int SLOW_SPEED   = 100;       // Slower speed when approaching obstacle
const int SONAR_THRESHOLD  = 60;    // First sonar threshold distance in centimeters
const int SONAR_THRESHOLD2 = 30;    // Second sonar threshold distance in centimeters
const int SONAR_THRESHOLD3 = 15;    // Third sonar threshold distance in centimeters


//******************************************************************************
// Motor control and related variables
//******************************************************************************
extern AF_MotorShield2 motorController;
extern AF_DCMotor2 leftMotor;
extern AF_DCMotor2 rightMotor;
extern bool isMoving;               // Indicates if moving
extern bool goingSlow;              // Indicates moving at slow speed


//******************************************************************************
// Sensors and related variables
//******************************************************************************
extern Servo panServo;              // Sewrvo for panning the ultrasonic sensor left and right
extern SonarSensor sonar;           // Ultrasonic sensor on Arduino
extern IRProximitySensor proxFront; // Front IR Proximity sensor (for obstacle detection ahead)
extern IRProximitySensor proxRight; // Right IR Proximity sensor (for obstacle detection on the right side)
extern IRProximitySensor proxLeft;  // Left IR Proximity sensor (for obstacle detection on the left side)
extern IRProximitySensor proxStep;  // Step IR Proximity sensor  (for step, i.e. drop-off, detection)
extern IRSensor ir;                 // IR Remote Sensor (for receiving signals from IR remote)
extern IMU imu;                     // Inertial Measurement Unit (IMU) module


//******************************************************************************
// TaskScheduler and Task variables
//******************************************************************************
extern TaskScheduler scheduler;
extern Task CheckRemoteTask;
extern Task CheckMagCalibrationTask;
extern Task CheckStepSensorTask;
extern Task CheckFrontSensorTask;
extern Task CheckSideSensorsTask;
extern Task CheckSonarSensorTask;
extern Task GoForwardTask;
#endif


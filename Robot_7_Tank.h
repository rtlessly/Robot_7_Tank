#ifndef _Robot_7_Tank_h_
#define _Robot_7_Tank_h_

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
// Sonar scan results
//******************************************************************************
struct SonarScanResults
{
    char BestDirection;     // Either 'L' (left) or 'R' (right)
    float LeftSum;          // Sum of left side
    float RightSum;         // Sum of right side
    float LeftMax;          // Max on left side
    float RightMax;         // Max on right side
};


//******************************************************************************
// Forward declarations
//******************************************************************************
bool CheckRemoteCommand();
bool CheckStepSensor();
bool CheckSideSensors();
bool CheckSonarSensor();
bool CheckCornered();
bool AvoidObstacleDetectedBySonar();

void Stop();
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

void PerformMagCalibration();

void BlinkLEDCount(uint16_t count, uint16_t onTime = 100, uint16_t offTime = 100);
void IndicateFailure(const __FlashStringHelper* msg = nullptr);


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
// Motor control and related variables
//******************************************************************************
extern AF_MotorShield2 motorController;
extern AF_DCMotor2 leftMotor;
extern AF_DCMotor2 rightMotor;
extern bool motorsEnabled;          // Indicates if the motors are enabled


//******************************************************************************
// Sensors and related variables
//******************************************************************************
extern Servo panServo;              // Sewrvo for panning the ultrasonic sensor left and right
extern SonarSensor sonar;           // Ultrasonic sensor on Arduino
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
extern Task CheckSideSensorsTask;
extern Task CheckSonarSensorTask;
extern Task CheckCorneredTask;
extern Task GoForwardTask;
#endif


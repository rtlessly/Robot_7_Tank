#ifndef _QuaternionFilters_h_
#define _QuaternionFilters_h_

#include "RTL_IMU.h"

// Functions defined in quaternionFilters.cpp
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltaT);
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltaT);
Vector3F MahonyQuaternionUpdate(const Vector3F& a, const Vector3F& g, const Vector3F& m, const float deltaT);
void UpdateEulerAngles();

// Data defined in quaternionFilters.cpp
extern float q[4];              // Quaternion vector
extern float pitch, yaw, roll;  // Euler angles
extern float dax, day, daz;     // dynamic acceleration
extern EulerAngles eulerAngles;

#endif


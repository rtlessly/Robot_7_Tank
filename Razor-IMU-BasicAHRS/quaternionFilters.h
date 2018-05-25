#ifndef _QuaternionFilters_h_
#define _QuaternionFilters_h_


struct EulerAngles
{
    float Yaw   { 0 };
    float Pitch { 0 };
    float Roll  { 0 };
};


// Functions defined in quaternionFilters.cpp
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltaT);
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltaT);
void UpdateEulerAngles();

// Data defined in quaternionFilters.cpp
extern float q[4];              // Quaternion vector
extern float pitch, yaw, roll;  // Euler angles
extern EulerAngles eulerAngles;

#endif


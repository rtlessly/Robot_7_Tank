#ifndef _QuaternionFilters_h_
#define _QuaternionFilters_h_


struct RawData
{
    int16_t X;
    int16_t Y;
    int16_t Z;
    RawData() : X(0), Y(0), Z(0) {};
    RawData(int16_t x, int16_t y, int16_t z) : X(x), Y(y), Z(z) {};
};


struct ScaledData
{
    float X;
    float Y;
    float Z;
    ScaledData() : X(0), Y(0), Z(0) {};
    ScaledData(float x, float y, float z) : X(x), Y(y), Z(z) {};
};


struct EulerAngles
{
    float Yaw   { 0 };
    float Pitch { 0 };
    float Roll  { 0 };
};


// Functions defined in quaternionFilters.cpp
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltaT);
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltaT);
ScaledData MahonyQuaternionUpdate(const ScaledData& a, const ScaledData& g, const ScaledData& m, const float deltaT);
void UpdateEulerAngles();

// Data defined in quaternionFilters.cpp
extern float q[4];              // Quaternion vector
extern float pitch, yaw, roll;  // Euler angles
extern float dax, day, daz;     // dynamic acceleration
extern EulerAngles eulerAngles;

#endif


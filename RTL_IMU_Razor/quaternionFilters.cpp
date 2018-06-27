#include <Arduino.h>
#include "QuaternionFilters.h"


// Quaternion vector used by both algorithms
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  

// The euler angles updated by both algorithms
float pitch, yaw, roll;
EulerAngles eulerAngles;

// Dynamic acceleration vector (true acceleration in Razor frame after gravity compensation)
float dax, day, daz;

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)


float old_invSqrt(float x)
{
   uint32_t i = 0x5F1F1412 - (*(uint32_t*)&x >> 1);
   float tmp = *(float*)&i;

   return tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
}


float invSqrt(float x)
{
    const float threehalfs = 1.5F;

    float half_x = x * 0.5F;
    float y = x;
    long i = *(long *)&y;                       // evil floating point bit level hacking

    i = 0x5f3759df - (i >> 1);                  // Use magic number to yield first approximation 
    y = *(float *)&i;                           // Convert approximation back to float
    y = y * (1.5F - (half_x * y * y));          // One iteration of newton's method to refine result
    //y  = y * ( 1.5F - ( x2 * y * y ) );         // 2nd iteration, usually not needed

    return y;
}


// There is a trade-off in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value


//******************************************************************************
// Implementation of Sebastian Madgwick's "...efficient orientation filter for... 
// inertial/magnetic sensor arrays" (see http://www.x-io.co.uk/category/open-source/ 
// for examples and more details) which fuses acceleration, rotation rate, and 
// magnetic moments to produce a quaternion-based estimate of absolute device 
// orientation -- which can be converted to yaw, pitch, and roll. Useful for 
// stabilizing quadcopters, etc. The performance of the orientation filter is at 
// least as good as conventional Kalman-based filtering algorithms but is much 
// less computationally intensive---it can be performed on a 3.3 V Pro Mini operating 
// at 8 MHz!
//******************************************************************************
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat)
{
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    if (ax == 0.0f && ay == 0.0f && az == 0.0f) return; // handle NaN

    if (mx == 0.0f && my == 0.0f && mz == 0.0f) return; // handle NaN

    // Normalise accelerometer measurement
    norm = sqrtf(ax * ax + ay * ay + az * az); 
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrtf(mx * mx + my * my + mz * mz);
    norm = 1.0f/norm;
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * mx;
    _2q1my = 2.0f * q1 * my;
    _2q1mz = 2.0f * q1 * mz;
    _2q2mx = 2.0f * q2 * mx;
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    
    // normalise step magnitude
    norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);
    norm = 1.0f/norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * deltat;
    q2 += qDot2 * deltat;
    q3 += qDot3 * deltat;
    q4 += qDot4 * deltat;
    
    norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}


//*****************************************************************************************
// Mahony filter fusion scheme. 
// Similar to Madgwick scheme but uses proportional and integral filtering
// (i.e., PID) on the error between estimated reference vectors and measured ones.
//
// These are the free parameters in the Mahony filter and fusion scheme. 
//    Kp for proportional feedback
//    Ki for integral feedback
//*****************************************************************************************

#define Kp 2.0f * 5.0f 
#define Ki 0.0f

// vector to hold integral error for Mahony method
float eInt[3] = {0.0f, 0.0f, 0.0f};

void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat)
{
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
    float norm;
    float hx, hy, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;
    float pa, pb, pc;

    // Auxiliary variables to avoid repeated arithmetic
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    if (ax == 0.0f && ay == 0.0f && az == 0.0f) return; // handle NaN

    if (mx == 0.0f && my == 0.0f && mz == 0.0f) return; // handle NaN

    // Normalise accelerometer measurement
    norm = sqrtf(ax * ax + ay * ay + az * az);
    norm = 1.0f / norm;        // use reciprocal for division
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrtf(mx * mx + my * my + mz * mz);
    norm = 1.0f / norm;        // use reciprocal for division
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
    hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
    bx = sqrtf((hx * hx) + (hy * hy));
    bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

    // Estimated direction of gravity
    vx = 2.0f * (q2q4 - q1q3);
    vy = 2.0f * (q1q2 + q3q4);
    vz = q1q1 - q2q2 - q3q3 + q4q4;

    // Compute true dynamic acceleration
    dax = ax - vx;
    day = ay - vy;
    daz = az - vz;

    // Estimated direction of magnetic field
    wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
    wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
    wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

    // Error is cross product between estimated direction and measured direction of gravity
    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
    
    if (Ki > 0.0f)
    {
        eInt[0] += ex;      // accumulate integral error
        eInt[1] += ey;
        eInt[2] += ez;
    }
    else
    {
        eInt[0] = 0.0f;     // prevent integral wind up
        eInt[1] = 0.0f;
        eInt[2] = 0.0f;
    }

    // Apply feedback terms
    gx = gx + Kp * ex + Ki * eInt[0];
    gy = gy + Kp * ey + Ki * eInt[1];
    gz = gz + Kp * ez + Ki * eInt[2];

    // Integrate rate of change of quaternion
    pa = q2;
    pb = q3;
    pc = q4;
    q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
    q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
    q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
    q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

    // Normalise quaternion
    norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    norm = 1.0f / norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}



Vector3F MahonyQuaternionUpdate(const Vector3F& a, const Vector3F& g, const Vector3F& m, const float deltaT)
{
    if (a.x == 0.0f && a.y == 0.0f && a.z == 0.0f) return g; // handle NaN

    if (m.x == 0.0f && m.y == 0.0f && m.z == 0.0f) return g; // handle NaN

    // Auxiliary variables to avoid repeated arithmetic
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];   // short name local variable for readability
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    float norm;

    // Normalise accelerometer measurement
    norm = invSqrt(a.x * a.x + a.y * a.y + a.z * a.z);

    auto ax = a.x * norm;
    auto ay = a.y * norm;
    auto az = a.z * norm;

    // Normalise magnetometer measurement
    norm = invSqrt(m.x * m.x + m.y * m.y + m.z * m.z);
    
    auto mx = m.x * norm;
    auto my = m.y * norm;
    auto mz = m.z * norm;

    // Reference direction of Earth's magnetic field
    auto hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    auto hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    auto bx = sqrtf((hx * hx) + (hy * hy));
    auto bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

    // Estimated direction of gravity
    auto vx = 2.0f * (q1q3 - q0q2);
    auto vy = 2.0f * (q0q1 + q2q3);
    auto vz = 2.0f * (q0q0 - 0.5f + q3q3);

    // Estimated direction of magnetic field
    auto wx = 2.0f * (bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2));
    auto wy = 2.0f * (bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3));
    auto wz = 2.0f * (bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2));

    // Error is sum of cross product between estimated direction and measured direction of field vectors
    auto ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    auto ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    auto ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

    if (Ki > 0.0f)
    {
        //eInt[0] += ex;      // accumulate integral error
        //eInt[1] += ey;
        //eInt[2] += ez;
        eInt[0] += ex * deltaT;      // accumulate integral error
        eInt[1] += ey * deltaT;
        eInt[2] += ez * deltaT;
    }
    else
    {
        eInt[0] = 0.0f;     // prevent integral wind up
        eInt[1] = 0.0f;
        eInt[2] = 0.0f;
    }

    // Apply feedback terms to gyro rates
    auto gx = g.x + Kp * ex + Ki * eInt[0];
    auto gy = g.y + Kp * ey + Ki * eInt[1];
    auto gz = g.z + Kp * ez + Ki * eInt[2];

    // Integrate rate of change of quaternion
    //auto pa = q1;
    //auto pb = q2;
    //auto pc = q3;
    auto qa = q0;
    auto qb = q1;
    auto qc = q2;

    q0 += (-qb * gx - qc * gy - q3 * gz) * (0.5f * deltaT);
    q1 += ( qa * gx + qc * gz - q3 * gy) * (0.5f * deltaT);
    q2 += ( qa * gy - qb * gz + q3 * gx) * (0.5f * deltaT);
    q3 += ( qa * gz + qb * gy - qc * gx) * (0.5f * deltaT);

    // Normalise quaternion
    norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    //norm = 1.0f / norm;
    q[0] = q0 * norm;
    q[1] = q1 * norm;
    q[2] = q2 * norm;
    q[3] = q3 * norm;

    // Compute true dynamic acceleration
    dax = a.x - vx;
    day = a.y - vy;
    daz = a.z - vz;

    return Vector3F(gx, gy, gz);
}


//******************************************************************************
// Define output variables from updated quaternion---these are Tait-Bryan angles, 
// commonly used in aircraft orientation. In this coordinate system:
//
// * The positive z-axis is down toward Earth
// * Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North 
//   if corrected for local declination), looking down on the sensor positive yaw 
//   is counterclockwise.
// * Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth 
//   is positive, up toward the sky is negative.
// * Roll is angle between sensor y-axis and Earth ground plane, y-axis up is 
//   positive roll.
//
// These arise from the definition of the homogeneous rotation matrix constructed 
// from quaternions.
// Tait-Bryan angles as well as Euler angles are non-commutative; that is, to get 
// the correct orientation the rotations must be applied in the correct order 
// which, for this configuration, is yaw, pitch, and then roll. For more see 
// http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles 
// which has additional links.
//
// yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
// pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
// roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
//******************************************************************************
void UpdateEulerAngles()
{
  float ysqr = q[2] * q[2];
  float t0 = 2.0 * (q[0] * q[1] + q[2] * q[3]);
  float t1 = 1.0 - 2.0 * (q[1] * q[1] + ysqr);
  float t2 = 2.0 * (q[0] * q[2] - q[1] * q[3]);
  float t3 = 2.0 * (q[0] * q[3] + q[1] * q[2]);
  float t4 = 1.0 - 2.0 * (ysqr + q[3] * q[3]);

  // Keep t2 within range of asin(-1, 1)
  t2 = t2 >  1.0 ?  1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  
  // Compute Euler angles in degrees
  auto yaw = (atan2(t3, t4) * 180.0f / PI); // -IMU_MAG_CORRECTION;
  auto pitch = asin(t2) * 180.0f / PI;
  auto roll  = atan2(t0, t1) * 180.0f / PI;
  
  noInterrupts();
  eulerAngles.Yaw   = yaw;
  eulerAngles.Pitch = pitch;
  eulerAngles.Roll  = roll;
  interrupts();
}


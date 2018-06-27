/*******************************************************************************
RTL_IMU implementation.

By      : R.T.Lessly
Date    : 2018-05-25
License : TBD
*******************************************************************************/
#ifndef _RazorIMU_h_
#define _RazorIMU_h_

#include <RTL_Math.h>

// I2C Address that the IMU responds to
#define RAZOR_IMU_ADDRESS ((byte)0x40)

// Device ID returned by the IMU when requested by command 'I'
#define RAZOR_IMU_ID ((byte)0x50)

// Standard 1-G acceleration value in meters per second^2
const float ONE_G = 9.80665;

/*******************************************************************************
Command Codes for IMU
All commands begin with "#" and is followed by a letter indicating the requested
operation.The command codes are:

- a : Return 3-axis acceleration vector in g's
- A : Return 3-axis dynamic acceleration vector in g's (gravity component removed)
- g : Return 3-axis gyroscope rate vector in radians / sec
- G : Return 3-axis integrated gyroscope gimbal angles in degrees
- m : Return 3-axis magnetometer vector in milliGuass
- V : Return 3-axis integrated velocity vector in meters / sec
- E : Return Euler angles as yaw, pitch, roll in degrees
- h : Returns the compass heading in degrees (0 = North, 90 = East, 180 = South, 270 = West)
- I : Returns the device ID(0x50)
- r : Returns the ready status of the IMU; 1 = ready, 0 = not ready
- c : Turns continuous mode on / off; must be followed by '1' (on) or '0' (off).
      This turns on / off all continuous reporting.To get continuous reporting
      of any values you have to issue both this command and the next command.
- cX: Turns continuous mode on / off for a specific value. The 'X' should
      be replaced with one of the value commands above(a, A, g, G, m, V, E,
      or h), and then followed by '1' (on) or '0' (off).For example, the
      command "ch1" turns on continuous reporting of compass heading, while
      "ch0" turns it off.

For example, to request Euler angles, send the command "QE" via either I2C or
the serial port.The command sequence "c1" followed by "cE1" turns on continuous
reporting of Euler angles.

Note: Continuous mode is not available via the I2C interface, it only works via 
      the serial port.
*******************************************************************************/

const char CMD_PREFIX = '#';
const char CMD_IS_READY = 'r';
const char CMD_ID = 'I';
const char CMD_ACCEL = 'a';
const char CMD_DYN_ACCEL = 'A';
const char CMD_GYRO = 'g';
const char CMD_GIMBAL = 'G';
const char CMD_MAG = 'm';
const char CMD_VELOCITY = 'V';
const char CMD_EULER = 'E';
const char CMD_CONTINUOS = 'c';
const char CMD_ZERO = 'z';
const char CMD_HEADING = 'h';


//struct RawData 
//{
//    int16_t X;
//    int16_t Y;
//    int16_t Z;
//    RawData() : X(0), Y(0), Z(0) {};
//    RawData(int16_t x, int16_t y, int16_t z) : X(x), Y(y), Z(z) {};
//};

//struct Vector3F : Vector3<float>
//{
//    float X;
//    float Y;
//    float Z;
//    Vector3F() : X(0), Y(0), Z(0) {};
//    Vector3F(float x, float y, float z) : X(x), Y(y), Z(z) {};
//};


struct EulerAngles
{
    float Yaw{ 0 };
    float Pitch{ 0 };
    float Roll{ 0 };
};


#endif

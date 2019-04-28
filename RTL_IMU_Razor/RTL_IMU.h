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
#define RAZOR_IMU_ADDRESS ((uint8_t)0x40)

// Device ID returned by the IMU when requested by command 'I'
#define RAZOR_IMU_ID ((uint8_t)0x50)

// Standard 1-G acceleration value in meters per second^2
const float ONE_G = 9.80665;


struct EulerAngles
{
    float Yaw;
    float Pitch;
    float Roll;

    EulerAngles() : Yaw(0), Pitch(0), Roll(0) {}
    EulerAngles(float yaw, float pitch, float roll) : Yaw(yaw), Pitch(pitch), Roll(roll) {}
};


/*******************************************************************************
Command Codes for IMU
All commands begin with "." and are followed by a letter indicating the requested
operation.The command codes are:

- a : Return 3-axis acceleration vector in g's (as Vector3F)
- A : Return raw 3-axis acceleration sensor data (as Vector3I)
- g : Return 3-axis gyroscope rate vector in radians / sec (as Vector3F)
- G : Return raw 3-axis gyroscope sensor data (as Vector3I)
- m : Return 3-axis magnetometer vector in milliGuass (as Vector3F)
- M : Return raw 3-axis magnetometer sensor data (as Vector3I)
- E : Return Euler angles as yaw, pitch, roll in degrees
- h : Returns the compass heading in degrees (0 = North, 90 = East, 180 = South, 270 = West)
- v : Returns the integrated velocity vector in m/s
- I : Returns the device ID(0x50)
- r : Returns the ready status of the IMU; 1 = ready, 0 = not ready
- c : Turns continuous mode on / off; must be followed by '1' (on) or '0' (off).
- x : Forces a reset of the IMU
      This turns on / off all continuous reporting.To get continuous reporting
      of any values you have to issue both this command and the next command.
- cX: Turns continuous mode on / off for a specific value. The 'X' should
      be replaced with one of the value commands above(a, A, g, G, m, V, E,
      or h), and then followed by '1' (on) or '0' (off).For example, the
      command ".ch1" turns on continuous reporting of compass heading, while
      ".ch0" turns it off.

For example, to request Euler angles, send the command ".E" via either I2C or
the serial port.The command sequence ".c1" followed by ".cE1" turns on continuous
reporting of Euler angles.

Note: Continuous mode is not available via the I2C interface, it only works via 
      the serial port.
*******************************************************************************/

const char PREFIX_RESPONSE = '#';
const char PREFIX_ERR = '!';
const char PREFIX_CMD = '.';

const char CMD_IS_READY = 'r';
const char CMD_ID = 'I';
const char CMD_ACCEL = 'a';
const char CMD_ACCEL_RAW = 'A';
const char CMD_GYRO = 'g';
const char CMD_GYRO_RAW = 'G';
const char CMD_MAG = 'm';
const char CMD_MAG_RAW = 'M';
const char CMD_EULER = 'E';
const char CMD_HEADING = 'h';
const char CMD_VELOCITY = 'v';

const char CMD_CONTINUOS = 'c';
const char CMD_ZERO = 'z';
const char CMD_SETTING = 's';
const char CMD_RESET = 'x';
const char CMD_DEBUG_I2C = 'd';

const char SETTING_MAG_BIAS = 'm';
const char SETTING_X = 'x';
const char SETTING_Y = 'y';
const char SETTING_Z = 'z';

const uint8_t REG_CONTROL = 0x01;
const uint8_t REG_CONTROL_LEN = sizeof(uint8_t);
const uint8_t REG_IS_READY = REG_CONTROL;
const uint8_t REG_IS_READY_LEN = REG_CONTROL_LEN;
const uint8_t REG_ID = REG_CONTROL + REG_CONTROL_LEN;
const uint8_t REG_ID_LEN = sizeof(uint8_t);

const uint8_t REG_ACCEL = REG_ID + REG_ID_LEN;
const uint8_t REG_ACCEL_LEN = sizeof(float) * 3;
const uint8_t REG_GYRO = REG_ACCEL + REG_ACCEL_LEN;
const uint8_t REG_GYRO_LEN = sizeof(float) * 3;
const uint8_t REG_MAG = REG_GYRO + REG_GYRO_LEN;
const uint8_t REG_MAG_LEN = sizeof(float) * 3;

const uint8_t REG_ACCEL_RAW = REG_MAG + REG_MAG_LEN;
const uint8_t REG_ACCEL_RAW_LEN = sizeof(int16_t) * 3;
const uint8_t REG_GYRO_RAW = REG_ACCEL_RAW + REG_ACCEL_RAW_LEN;
const uint8_t REG_GYRO_RAW_LEN = sizeof(int16_t) * 3;
const uint8_t REG_MAG_RAW = REG_GYRO_RAW + REG_GYRO_RAW_LEN;
const uint8_t REG_MAG_RAW_LEN = sizeof(int16_t) * 3;

const uint8_t REG_EULER = REG_MAG_RAW + REG_MAG_RAW_LEN;
const uint8_t REG_EULER_LEN = sizeof(float) * 3;
const uint8_t REG_HEADING = REG_EULER + REG_EULER_LEN;
const uint8_t REG_HEADING_LEN = sizeof(float);

const uint8_t REG_MAGBIAS_X = REG_HEADING + REG_HEADING_LEN;
const uint8_t REG_MAGBIAS_X_LEN = sizeof(int16_t);
const uint8_t REG_MAGBIAS_Y = REG_MAGBIAS_X + REG_MAGBIAS_X_LEN;
const uint8_t REG_MAGBIAS_Y_LEN = sizeof(int16_t);
const uint8_t REG_MAGBIAS_Z = REG_MAGBIAS_Y + REG_MAGBIAS_Y_LEN;
const uint8_t REG_MAGBIAS_Z_LEN = sizeof(int16_t);

const uint8_t REG_VELOCITY = REG_MAGBIAS_Z + REG_MAGBIAS_Z_LEN;
const uint8_t REG_VELOCITY_LEN = sizeof(float)*3;

const char ERR_BAD_CMD_STRING = 0x81;
const char ERR_CMD_UNRECOGNIZED = 0x82;

#endif

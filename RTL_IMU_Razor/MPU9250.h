#ifndef _MPU9250_h_
#define _MPU9250_h_

#include <inttypes.h>
#include <Streaming.h>
#include "quaternionFilters.h"

// Magnetometer bias correction values, in raw data units
// Subtract the bias values from the raw data reading for each axis
// before doing anything else with the data values.
// These were empirically determined using a separate calibration program, and are different for every sensor!
#define MAG_BIASX -109.11  // Default to 0 if unknown
#define MAG_BIASY  272.19  // Default to 0 if unknown
#define MAG_BIASZ  136     // Default to 0 if unknown

// Magnetometer scale correction values
// Multiply the bias-corrected values by this scale factor to even out
// the scaling differences between axes. Do this before converting the mag
// values to milliGaus.
// These were empirically determined using a separate calibration program, and are different for every sensor!
#define MAG_SCALEX 1.00  // Default to 1.0 if unknown (DO NOT SET TO 0.0!!)
#define MAG_SCALEY 1.00  // Default to 1.0 if unknown (DO NOT SET TO 0.0!!)
#define MAG_SCALEZ 1.00  // Default to 1.0 if unknown (DO NOT SET TO 0.0!!)

// Correction for difference between magnetic north and true north (degrees)
// Different for every location on earth (see ngdc.noaa.gov). 
#define IMU_MAG_CORRECTION  3.25   // Declination at Dallas, TX is 3.25 degrees on 2017-05-01 
//#define IMU_MAG_CORRECTION 13.8  // Declination at Danville, CA is 13.8 degrees (13 degrees 48 minutes and 47 seconds) on 2014-04-04



class MPU9250
{
    public: MPU9250();

    public: void Begin(Print& stream=NullPrint);

    public: void SelfTest(float* pResuts);

    public: void Calibrate(Print& stream = NullPrint);

    public: void CalibrateFine(Print& stream);

    public: bool TestConnection(Print& stream=NullPrint);

    public: int8_t Update();

    public: float UpdateCompassHeading();

    public: float GetHeading() { return heading; };

    public: Vector3I GetAccelRaw() { return acclRaw; };

    public: Vector3I GetGyroRaw() { return gyroRaw; };

    public: Vector3I GetMagRaw() { return magRaw; };

    public: Vector3F GetAccel() { return accl; };

    public: Vector3F GetGyro() { return gyro; };

    public: Vector3F GetMag() { return mag; };

    public: Vector3F GetSmoothedAccel() { return lastAccl; };

    public: int16_t ReadTempData();

    public: float GetMres();

    public: float GetGres();

    public: float GetAres();

    public: Vector3F GetAccelCalibration() { return acclBias; };

    public: Vector3F GetGyroCalibration() { return gyroBias; };

    public: void GetMagCalibration(float bias[3], float sensitivity[3]);

    public: void SetMagBiasX(int16_t bias) { magBias[0] = bias; };

    public: void SetMagBiasY(int16_t bias) { magBias[1] = bias; };

    public: void SetMagBiasZ(int16_t bias) { magBias[2] = bias; };

    private: void initMPU9250();

    private: void initAK8963();


    private: void SetAcclBias(Vector3F & bias);

    private: void SetGyroBias(Vector3F & bias);

    private: static void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);

    private: static uint8_t readByte(uint8_t address, uint8_t subAddress);

    private: static void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);


    private: Vector3I acclRaw;                  // Raw accelerometer vector
    private: Vector3I gyroRaw;                  // Raw gyro rates vector
    private: Vector3I magRaw;                   // Raw magnetometer vector
    private: Vector3F accl;                     // Scaled accelerometer vector (g)
    private: Vector3F gyro;                     // Scaled gyro rates (radians/sec)
    private: Vector3F mag;                      // Scaled magnetometer vector (milliGaus)
    private: float heading;                     // Compass heading (degrees)

    private: Vector3F lastMag{ 0.0, 0.0,0.0 };  // The smoothed magnetometer reading from the previous iteration
    private: Vector3F lastAccl{ 0.0, 0.0,0.0 }; // The smoothed accelerometer reading from the previous iteration

    private: uint8_t Ascale;                    // acceleromter scale setting
    private: uint8_t Gscale;                    // gyroscope scale setting
    private: uint8_t Mscale;                    // magentometer scale setting - Choose either 14-bit or 16-bit resolution
    private: uint8_t Mmode;                     // magentometer scale MODE - 2=8 Hz, 6=100 Hz continuous magnetometer data read
    private: Vector3F acclBias;                 // Bias corrections for accelerometer
    private: Vector3F accBiasFine;              // Fine bias corrections for accelerometer
    private: Vector3F acclThreshold;            // Zero threshold for acceleration
    private: Vector3F gyroBias;                 // Bias corrections for gyro
    private: float   magBias[3] { 0, 0, 0 };    // Magnetometer bias calibration (milliGauss)
    private: float   magSens[3] { 0, 0, 0 };    // Magnetometer sensitivity calibration (milliGauss)
};

#endif

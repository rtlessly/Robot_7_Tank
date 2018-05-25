// RazorIMU.h
#ifndef _RazorIMU_h_
#define _RazorIMU_h_

#include <inttypes.h>
#include <Streaming.h>
#include "quaternionFilters.h"

// Magnetometer bias correction values, in raw data units
// Subtract the bias values from the raw data reading for each axis
// before doing anything else with the data values.
// These were empirically determined using a separate calibration program, and are different for every sensor!
#define MAG_BIASX -108   // Default to 0 if unknown
#define MAG_BIASY 78     // Default to 0 if unknown
#define MAG_BIASZ 22     // Default to 0 if unknown

// Magnetometer scale correction values
// Multiply the bias-corrected values by this scale factor to even out
// the scaling differences between axes. Do this before converting the mag
// values to milliGaus.
// These were empirically determined using a separate calibration program, and are different for every sensor!
#define MAG_SCALEX 0.99  // Default to 1.0 if unknown (DO NOT SET TO 0.0!!)
#define MAG_SCALEY 1.05  // Default to 1.0 if unknown (DO NOT SET TO 0.0!!)
#define MAG_SCALEZ 0.97  // Default to 1.0 if unknown (DO NOT SET TO 0.0!!)

struct RawData
{
    int16_t X;
    int16_t Y;
    int16_t Z;
    RawData() : X(0), Y(0), Z(0) {};
};


struct ScaledData
{
    float X;
    float Y;
    float Z;
    ScaledData() : X(0), Y(0), Z(0) {};
};


class MPU9250
{
    public: MPU9250();

    public: bool TestConnection(Print& stream=NullPrint);

    public: void Initialize(Print& stream=NullPrint);

    public: void SelfTest(float* pResuts);

    public: void Calibrate(Print& stream = NullPrint);

    public: bool DataReady();

    public: int8_t Update();

    public: void GetAccelRaw(RawData& accelData);

    public: void GetGyroRaw(RawData& gyroData);

    public: void GetMagRaw(RawData& magData);

    public: void GetAccel(ScaledData& accelData);

    public: void GetGyro(ScaledData& gyroData);

    public: void GetMag(ScaledData& magData);

    public: int16_t ReadTempData();

    public: float GetMres();

    public: float GetGres();

    public: float GetAres();

    public: void GetAccelCalibration(float bias[3]);

    public: void GetGyroCalibration(float bias[3]);

    public: void GetMagCalibration(float bias[3], float sensitivity[3]);


    private: void initMPU9250();

    private: void initAK8963();

    private: static void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);

    private: static uint8_t readByte(uint8_t address, uint8_t subAddress);

    private: static void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);

    
    private: uint8_t Ascale;                     // acceleromter scale setting
    private: uint8_t Gscale;                     // gyroscope scale setting
    private: uint8_t Mscale;                     // magentometer scale setting - Choose either 14-bit or 16-bit resolution
    private: uint8_t Mmode;                      // magentometer scale MODE - 2=8 Hz, 6=100 Hz continuous magnetometer data read

    private: float   gyroBias[3] = { 0, 0, 0 };  // Bias corrections for gyro
    private: float   accelBias[3] = { 0, 0, 0 }; // Bias corrections for accelerometer
    private: float   magBias[3] = { 0, 0, 0 };   // Magnetometer bias calibration (milliGauss)
    private: float   magSens[3] = { 0, 0, 0 };   // Magnetometer sensitivity calibration (milliGauss)

    private: RawData accelRaw;
    private: RawData gyroRaw;
    private: RawData magRaw;
};



//bool RazorIMU_TestConnection();
//void RazorIMU_SelfTest(float* pResuts);
//void RazorIMU_Calibrate();
//void RazorIMU_Initialize(float* magCalibration);
//bool RazorIMU_Ready();
//
//void initMPU9250();
//void initAK8963(float* destination);
//
//
//float RazorIMU_GetMres();
//
//float RazorIMU_GetGres();
//
//float RazorIMU_GetAres();
//
//void readAccelData(int16_t * destination);
//
//void readGyroData(int16_t * destination);
//
//void readMagData(int16_t * destination);
//
//int16_t readTempData();

#endif


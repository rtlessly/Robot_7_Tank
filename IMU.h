#ifndef _IMU_h
#define _IMU_h

#include <RTL_Stdlib.h>
#include <RTL_Math.h>
#include "RTL_IMU_Razor\RTL_IMU.h"


struct YawManouver
{
    YawManouver(int16_t angle);

    float GetYawRate();

    bool Update();

    private: float targetAngle;                              // Target angle
    private: float currAngle;                                // How much we have turned so far
    private: float w0;                                       // Initial gyro yaw reading
    private: uint32_t t0;                                    // Time of last measurement
};


class IMU
{
    /*--------------------------------------------------------------------------
    Intitialization
    --------------------------------------------------------------------------*/
    public: bool Begin();

    /*--------------------------------------------------------------------------
    Public interface
    --------------------------------------------------------------------------*/
    public: bool ValidateConnectivity(uint32_t timeout = 10000);
    public: bool WaitForReady(uint32_t timeout = 20000);
    public: Vector3F GetAccel();
    public: Vector3F GetGyro();
    public: Vector3F GetMag();
    public: Vector3I GetMagRaw();
    public: float GetCompassHeading();
    public: void SetMagBias(int16_t xBias, int16_t yBias);
    //public: void BeginYaw(int16_t angle);
    //public: bool UpdateYaw();

    /*--------------------------------------------------------------------------
    Properties
    --------------------------------------------------------------------------*/
    public: byte GetLastStatus() { return _i2cStatus; };
    private: byte _i2cStatus;

    public: bool IsActive() { return _isActive; };
    private: bool _isActive = false;

    /*--------------------------------------------------------------------------
    Internal implementation
    --------------------------------------------------------------------------*/
    private: byte SendCommand(uint8_t commandCode, uint8_t* response, size_t responseSize = 1);

    private: byte SendCommandWithData(uint8_t commandCode, uint8_t * data, size_t dataSize, uint8_t * response=nullptr, size_t responseSize=0);
};

#endif


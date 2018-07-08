#ifndef _IMU_h
#define _IMU_h

#include <RTL_Stdlib.h>
#include <RTL_Math.h>
#include "RTL_IMU_Razor\RTL_IMU.h"

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
    public: bool WaitForReady(uint32_t timeout = 10000);
    public: Vector3F GetAccel();
    public: Vector3F GetGyro();
    public: Vector3F GetMago();
    public: Vector3I GetMagRaw();
    public: float GetCompassHeading();
    public: void SetMagBias(int16_t xBias, int16_t yBias);

    public: byte GetLastStatus() { return _i2cStatus; };

    /*--------------------------------------------------------------------------
    Internal implementation
    --------------------------------------------------------------------------*/
    private: byte SendCommand(uint8_t commandCode, uint8_t* response, size_t responseSize = 1);

    private: byte SendCommandWithData(uint8_t commandCode, const uint8_t * data, size_t dataSize, uint8_t * response=nullptr, size_t responseSize=0);

    /*--------------------------------------------------------------------------
    Internal state
    --------------------------------------------------------------------------*/
    private: byte _i2cStatus;
};

#endif


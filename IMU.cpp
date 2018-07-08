
#define DEBUG 0

#include <arduino.h>
#include <Wire.h>
#include <RTL_I2C.h>
#include "IMU.h"


//******************************************************************************
// Performs IMU startup tasks
//******************************************************************************
bool IMU::Begin()
{
    // Validate connectivity to IMU
    if (!ValidateConnectivity()) return false;

    Logger() << F("IMU connectivity confirmed") << endl;

    // Wait for the IMU to be ready
    if (!WaitForReady()) return false;

    Logger() << F("IMU ready") << endl;

    return true;
}


Vector3F IMU::GetAccel()
{
    Vector3F data;

    SendCommand(REG_ACCEL, (byte*)&data, sizeof data);

    return data;
}


Vector3F IMU::GetGyro()
{
    Vector3F data;

    SendCommand(REG_GYRO, (byte*)&data, sizeof data);

    return data;
}


Vector3F IMU::GetMago()
{
    Vector3F data;

    SendCommand(REG_MAG, (byte*)&data, sizeof data);

    return data;
}


Vector3I IMU::GetMagRaw()
{
    Vector3I data;

    SendCommand(REG_MAG_RAW, (byte*)&data, sizeof data);

    return data;
}


float IMU::GetCompassHeading()
{
    float heading = 0.0;

    SendCommand(REG_HEADING, (byte*)&heading, sizeof heading);
    TRACE(Logger() << "GetCompassHeading: heading=" << heading << " (" << _HEX(*(uint32_t*)&heading) << ')' << endl);

    return heading;
}


void IMU::SetMagBias(int16_t xBias, int16_t yBias)
{
    SendCommandWithData(REG_MAGBIAS_X, (byte*)&xBias, sizeof xBias);
    SendCommandWithData(REG_MAGBIAS_Y, (byte*)&yBias, sizeof yBias);
}


//******************************************************************************
// Check IMU connectivity
//******************************************************************************
bool IMU::ValidateConnectivity(uint32_t timeout)
{
    byte imuID = 0;

    timeout += millis();

    while (imuID != RAZOR_IMU_ID && millis() < timeout)
    {
        if (SendCommand(REG_ID, &imuID) != 0) continue;

        Logger() << F("IMU ID Check:(expected,actual)=(") << _HEX(RAZOR_IMU_ID) << ',' << _HEX(imuID) << ')' << endl;
    }

    return (imuID == RAZOR_IMU_ID);
}


//******************************************************************************
// Wait the IMU to be ready
//******************************************************************************
bool IMU::WaitForReady(uint32_t timeout)
{
    byte imuReady = 0;

    timeout += millis();

    while (imuReady == 0 && millis() < timeout)
    {
        SendCommand(REG_IS_READY, &imuReady);
    }

    return (imuReady != 0);
}


//******************************************************************************
// Send standard 1-byte command to IMU via I2C
//******************************************************************************
byte IMU::SendCommand(uint8_t commandCode, uint8_t* response, size_t responseSize)
{
    _i2cStatus = I2C_SendMessage(RAZOR_IMU_ADDRESS, &commandCode, sizeof(commandCode), response, responseSize);

    if (_i2cStatus != 0)
    {
        Logger() << F("I2C communication failure with IMU; status=") << _i2cStatus << endl;
    }

    return _i2cStatus;
}


//******************************************************************************
// Send 1-byte command to IMU with data via I2C
//******************************************************************************
byte IMU::SendCommandWithData(uint8_t commandCode, const uint8_t* data, size_t dataSize, uint8_t* response, size_t responseSize)
{
    uint8_t message[10] = {commandCode};

    memcpy(&message[1], data, dataSize);

    _i2cStatus = I2C_SendMessage(RAZOR_IMU_ADDRESS, message, dataSize + 1, response, responseSize);

    if (_i2cStatus != 0)
    {
        Logger() << F("I2C communication failure with IMU; status=") << _i2cStatus << endl;
    }

    return _i2cStatus;
}

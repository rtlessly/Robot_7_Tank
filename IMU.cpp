
#define DEBUG 1

#include <arduino.h>
//#include <Wire.h>
//#include <RTL_I2C.h>
#include <RTL_I2C.h>
#include "IMU.h"

extern IMU imu;

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

    _isActive = true;
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


Vector3F IMU::GetMag()
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


//float targetAngle;                              // Target angle
//float currAngle;                                // How much we have turned so far
//float w0;                                       // Initial gyro yaw reading
//uint32_t t0;                                    // Time of last measurement
//
//
//void IMU::BeginYaw(int16_t angle)
//{
//    //GetGyro();                                  // Initial turn rate
//    targetAngle = abs(angle)*DEG_TO_RAD;        // Target angle
//    currAngle = 0.0f;                           // How much we have turned so far
//    t0 = micros();                              // Time of last measurement
//    w0 = GetGyro().z;                           // Initial gyro yaw reading
//}
//
//
//bool IMU::UpdateYaw()
//{
//    // Use absolute value of current angle since we only need to measure the magnitude 
//    // of the turn and not the direction
//    if (abs(currAngle) >= targetAngle) return true;
//
//    // Get current time and compute delta-T since last check
//    // Use UDIFF to compute difference of unsigned numbers (handles 32-bit wrap-around)
//    auto t1 = micros();
//    auto deltaT = UDIFF(t1, t0);
//
//    // Need at least 4ms between measurements to ensure we have an updated measurement
//    if (deltaT >= 4000)
//    {
//        // Get new gyro measurement and compute time since last measurement (in seconds)
//        auto w1 = GetGyro().z;
//        auto dt = deltaT / 1000000.0f;
//
//        // Update turn angle using trapezoidal integration
//        currAngle += (w0 + (w1 - w0) / 2.0) * dt;
//
//        TRACE(Logger(F("UpdateYaw")) << _FLOAT(dt, 6) << ',' << _FLOAT(w0, 3) << ',' << _FLOAT(w1, 3) << ',' << _FLOAT(currAngle, 3) << endl);
//
//        // Update starting values for next iteration
//        w0 = w1;
//        t0 = t1;
//    }
//
//    return false;
//}


//******************************************************************************
// Check IMU connectivity
//******************************************************************************
bool IMU::ValidateConnectivity(uint32_t timeout)
{
    uint8_t imuID = 0;

    timeout += millis();

    while (imuID != RAZOR_IMU_ID && millis() < timeout)
    {
        if (SendCommand(REG_ID, &imuID) != 0) continue;

        Logger() << F("IMU ID Check:(expected,actual)=(0x") << _HEX(RAZOR_IMU_ID) << ",0x" << _HEX(imuID) << ')' << endl;
    }

    return (imuID == RAZOR_IMU_ID);
}


//******************************************************************************
// Wait the IMU to be ready
//******************************************************************************
bool IMU::WaitForReady(uint32_t timeout)
{
    uint8_t imuReady = 0;

    timeout += millis();

    while (imuReady != 1 && millis() < timeout)
    {
        if (SendCommand(REG_IS_READY, &imuReady) != 0) continue;

        Logger() << F("IMU ready=") << imuReady << endl;
    }

    return (imuReady != 0);
}


//******************************************************************************
// Send standard 1-byte command to IMU via I2C
//******************************************************************************
byte IMU::SendCommand(uint8_t commandCode, uint8_t* response, size_t responseSize)
{
    _i2cStatus = I2c.read(RAZOR_IMU_ADDRESS, commandCode, responseSize, response);
    //_i2cStatus = I2C_SendRequest(RAZOR_IMU_ADDRESS, commandCode, response, responseSize);

    if (_i2cStatus != 0)
    {
        Logger() << F("I2C communication failure with IMU; status=") << _i2cStatus << endl;
    }

    return _i2cStatus;
}


//******************************************************************************
// Send 1-byte command to IMU with data via I2C
//******************************************************************************
byte IMU::SendCommandWithData(uint8_t commandCode, uint8_t* data, size_t dataSize, uint8_t* response, size_t responseSize)
{
    //uint8_t message[10] = {commandCode};

    //memcpy(&message[1], data, dataSize);

    //_i2cStatus = I2C_SendMessage(RAZOR_IMU_ADDRESS, message, dataSize + 1, response, responseSize);
    _i2cStatus = I2c.write(RAZOR_IMU_ADDRESS, commandCode, data, dataSize);

    if (_i2cStatus == 0 && response != nullptr)
    {
        _i2cStatus = I2c.read(RAZOR_IMU_ADDRESS, responseSize, response);
    }

    if (_i2cStatus != 0)
    {
        Logger() << F("I2C communication failure with IMU; status=") << _i2cStatus << endl;
    }

    return _i2cStatus;
}


YawManouver::YawManouver(int16_t angle)
{
    targetAngle = abs(angle)*DEG_TO_RAD;        // Target angle
    currAngle = 0.0f;                           // How much we have turned so far
    t0 = micros();                              // Time of last measurement
    w0 = GetYawRate();                          // Initial gyro yaw reading
}


bool YawManouver::Update()
{
    // Use absolute value of current angle since we only need to measure the magnitude 
    // of the turn and not the direction
    if (abs(currAngle) >= targetAngle) return true;

    // Get current time and compute delta-T since last check
    // Use UDIFF to compute difference of unsigned numbers (handles 32-bit wrap-around)
    auto t1 = micros();
    auto deltaT = UDIFF(t1, t0);

    // Need at least 10ms between measurements to ensure we have an updated measurement
    if (deltaT >= 10000)
    {
        // Get new gyro measurement and compute time since last measurement (in seconds)
        auto w1 = GetYawRate();
        auto dt = deltaT / 1000000.0f;

        // Update turn angle using trapezoidal integration
        currAngle += (w0 + (w1 - w0) / 2.0) * dt;

        TRACE(Logger(F("YawManouver::Update")) << _FLOAT(dt, 6) << ',' << _FLOAT(w0, 3) << ',' << _FLOAT(w1, 3) << ',' << _FLOAT(currAngle, 3) << endl);

        // Update starting values for next iteration
        w0 = w1;
        t0 = t1;
    }

    return false;
}


float YawManouver::GetYawRate()
{
    auto gyro = imu.GetGyro();

    while (isnan(gyro.z)) gyro = imu.GetGyro();

    TRACE(Logger(F("YawManouver::Gyro: ")) << F("x=") << _FLOAT(gyro.x, 3) << F(", y=") << _FLOAT(gyro.y, 3) << F(", z=") << _FLOAT(gyro.z, 3) << endl);

    return  gyro.z;
}



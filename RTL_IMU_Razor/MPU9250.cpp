/* *****************************************************************************
MPU-9250 IMU Sensor Basic Driver

The MPU-9250 consists of an MPU-6500 3-axis gyroscope/accelerometer and an AK8963 
3-axis magnetometer. This driver provides basic functionality for self-testing, 
calibration, initialization, and reading data from the MPU-9250. 

This driver includes a 9 DoF sensor fusion algorithm that can be used to improve 
accuracy and combat gyro drift. Two such algorithms are available: the open source 
Madgwick algorithm (similar to a Kalman filter) and the Mahony PID filter algorithm. 
Both algorithms make use of the magnetometer to correct for gyro drift.

The MPU-9250 is 3.3V I2C sensor and is not 5V tolerant. Therefore, it can only 
be directly connected to another 3.3V processor (such as the Arduino Pro Mini or 
Teensy 3.1), or to a bi-directional logic level shifter to convert signal levels 
between 3.3V and 5V for connection to 5V processors, such as the Arduinoi Uno.

This driver disables the internal pull-ups used by the Wire library in the 
Wire.h/twi.c utility file, so SDA and SCL should have external pull-up resistors 
to 3.3V. 

The sensor can also run in 400 kHz fast I2C mode if desired.

This driver is based on an extensively modified version of the freely reusable 
MPU-9250 example code developed by Kris Winer ((c) Kris Winer, April 1, 2014).
***************************************************************************** */

#include <Arduino.h>
#include <Wire.h>
#include <RTL_Stdlib.h>
#include "MPU9250RegisterMap.h"
#include "quaternionFilters.h"
#include "MPU9250.h"


float lowPass(float newValue, float oldValue, float alpha);
Vector3F lowPass(Vector3F& newValue, Vector3F& oldValue, float alpha);


MPU9250::MPU9250()
{
    Ascale = AFS_2G;            // Specify acceleromter scale setting
    Gscale = GFS_250DPS;        // Specify gyroscope scale setting
    Mscale = MFS_16BITS;        // Specify magentometer scale setting - Choose either 14-bit or 16-bit resolution
    Mmode  = 0x06;              // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
    
    // Set the raw mag bias values. These are empirically determined for each sensor (see note in MPU9250.h)
    magBias[0] = MAG_BIASX; 
    magBias[1] = MAG_BIASY; 
    magBias[2] = MAG_BIASZ;
}


bool MPU9250::TestConnection(Print& stream)
{
    // Read the WHO_AM_I register of MPU-9250, this is a good test of communication
    byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250

    stream << "MPU9250: I AM " << _HEX(c) << "; I should be " << _HEX(0x71) << endl;

    if (c != 0x71) // if WHO_AM_I returned incorrect value then abort
    {
        stream << "Could not connect to MPU9250 - incorrect chip ID read: " << _HEX(c) << endl;
        return false;
    }

    // Ensure MPU-9250 is configured in bypass mode so the AK8963 is connected to the I2C bus
    writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x02);

    // Read the WHO_AM_I register of AK8963, this is a good test of communication
    byte d = readByte(AK8963_ADDRESS, AK8963_WHO_AM_I);

    stream << endl;
    stream << "AK8963: I AM " << _HEX(d) << "; I should be " << _HEX(0x48) << endl;

    if (d != 0x48) // if WHO_AM_I returned incorrect value then abort
    {
        stream << "Could not connect to AK8963 - incorrect chip ID read: " << _HEX(d) << endl;
        return false;
    }

    return true;
}


void MPU9250::Begin(Print& stream)
{
    // Initialize MPU-9250 for active mode read of accelerometer, gyroscope, and temperature
    initMPU9250();
    stream << endl;
    stream << "MPU-9250 initialized for active data mode...." << endl;

    // Initialize AK8963 for active mode read of magnetometer
    initAK8963();

    stream << endl;
    stream << "AK8963 initialized for active data mode...." << endl;
}


// Accelerometer and gyroscope self test; check calibration wrt factory settings
// Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
void MPU9250::SelfTest(float* pResuts)
{
    uint8_t rawData[6] = { 0, 0, 0, 0, 0, 0 };
    uint8_t selfTest[6];
    int32_t gAvg[3] = { 0 }, aAvg[3] = { 0 }, aSTAvg[3] = { 0 }, gSTAvg[3] = { 0 };
    float factoryTrim[6];
    uint8_t FS = 0;

    writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);       // Set gyro sample rate to 1 kHz
    writeByte(MPU9250_ADDRESS, CONFIG, 0x02);           // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, FS << 3);   // Set full scale range for the gyro to 250 dps
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02);    // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, FS << 3);  // Set full scale range for the accelerometer to 2 g

    // get average current values of gyro and accelerometer
    for (int ii = 0; ii < 200; ii++)
    {
        readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers into data array
        aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value
        aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
        aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

        readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers sequentially into data array
        gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value
        gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
        gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
    }

    // Get average of 200 values and store as average current readings
    for (int ii = 0; ii < 3; ii++)
    {
        aAvg[ii] /= 200;
        gAvg[ii] /= 200;
    }

    // Configure the accelerometer for self-test
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0xE0);  // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
    delay(25);  // Delay a while to let the device stabilize

     // get average self-test values of gyro and accelerometer
    for (int ii = 0; ii < 200; ii++)
    {
        readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);           // Read the six raw data registers into data array
        aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);    // Turn the MSB and LSB into a signed 16-bit value
        aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
        aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

        readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);            // Read the six raw data registers sequentially into data array
        gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);    // Turn the MSB and LSB into a signed 16-bit value
        gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
        gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
    }

    // Get average of 200 values and store as average self-test readings
    for (int ii = 0; ii < 3; ii++)
    {
        aSTAvg[ii] /= 200;
        gSTAvg[ii] /= 200;
    }

    // Configure the gyro and accelerometer for normal operation
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);
    delay(25);  // Delay a while to let the device stabilize

    // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
    selfTest[0] = readByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL); // X-axis accel self-test results
    selfTest[1] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
    selfTest[2] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
    selfTest[3] = readByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO);  // X-axis gyro self-test results
    selfTest[4] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
    selfTest[5] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

    // Retrieve factory self-test value from self-test code reads
    factoryTrim[0] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[0] - 1.0))); // FT[Xa] factory trim calculation
    factoryTrim[1] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[1] - 1.0))); // FT[Ya] factory trim calculation
    factoryTrim[2] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[2] - 1.0))); // FT[Za] factory trim calculation
    factoryTrim[3] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[3] - 1.0))); // FT[Xg] factory trim calculation
    factoryTrim[4] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[4] - 1.0))); // FT[Yg] factory trim calculation
    factoryTrim[5] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[5] - 1.0))); // FT[Zg] factory trim calculation

    // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
    // To get percent, must multiply by 100
    for (int i = 0; i < 3; i++)
    {
        pResuts[i] = 100.0*((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i] - 100.;   // Report percent differences
        pResuts[i + 3] = 100.0*((float)(gSTAvg[i] - gAvg[i])) / factoryTrim[i + 3] - 100.; // Report percent differences
    }
}


// Calibrates gyro and accelerometer after device initialization.
// This calculates the average of the at-rest readings and then loads the
// resulting offsets into accelerometer and gyro bias registers.
void MPU9250::Calibrate(Print& stream)
{
    uint8_t data[12]; // data array for values to read/write to MPU9250

    // reset device
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
    delay(100);

    // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
    // else use the internal oscillator, bits 2:0 = 001
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
    writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
    delay(200);

    // Configure device for bias calculation
    writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
    writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
    writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
    writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
    writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
    delay(15);

    // Configure MPU6050 gyro and accelerometer for bias calculation
    writeByte(MPU9250_ADDRESS, CONFIG, 0x01);       // Set low-pass filter to 188 Hz
    writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);   // Set sample rate to 1 kHz
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

    const int32_t gyroSensitivity = 131;            // = 131 LSB/degrees/sec
    const int32_t acclSensitivity = 16384;          // = 16384 LSB/g
    const int countTrials = 5;

    acclBias = 0;
    gyroBias = 0;

    // Perform 5 iterations of calibration to get 5 sampples for averaging.
    for (auto trial = 0; trial < countTrials; trial++)
    {
        Vector3F accl_bias;
        Vector3F gyro_bias;
        auto count = 0;

        // Take 1 second of accelerometer and gyroscope readings (25 x 40ms)
        for (auto iter = 0; iter < 25; iter++)
        {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

            // Configure FIFO to capture accelerometer and gyro data for bias calculation
            writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);    // Enable FIFO
            writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);      // Enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU-9150)
            delay(40);                                      // accumulate 40 samples in 40 milliseconds = 480 bytes

            // At end of sample accumulation, turn off FIFO sensor read
            writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);            // Disable gyro and accelerometer sensors for FIFO
            readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count

            int fifo_count = ((uint16_t)data[0] << 8) | data[1];
            int packet_count = fifo_count / 12;             // Number of complete sets of gyro and accelerometer data

            for (auto ii = 0; ii < packet_count; ii++)
            {
                // read data for averaging
                readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]);

                // Form signed 16-bit integer for each sample in FIFO and sum to get
                // accumulated signed 32-bit biases
                auto accl_x = (int16_t)(((int16_t)data[0] << 8) | data[1]);
                auto accl_y = (int16_t)(((int16_t)data[2] << 8) | data[3]);
                auto accl_z = (int16_t)(((int16_t)data[4] << 8) | data[5]);
                auto gyro_x = (int16_t)(((int16_t)data[6] << 8) | data[7]);
                auto gyro_y = (int16_t)(((int16_t)data[8] << 8) | data[9]);
                auto gyro_z = (int16_t)(((int16_t)data[10] << 8) | data[11]);

                accl_bias.x += accl_x;
                accl_bias.y += accl_y;
                accl_bias.z += accl_z;
                gyro_bias.x += gyro_x;
                gyro_bias.y += gyro_y;
                gyro_bias.z += gyro_z;
                count++;
                TRACE(Logger() << count << ". accel[x,y,z]=[" << accl_x << ", " << accl_y << ", " << accl_z << "]" << endl);
                TRACE(Logger() << count << ". accl_bias[x,y,z]=[" << accl_bias.x << ", " << accl_bias.y << ", " << accl_bias.z << "]" << endl);
            }
        }

        // Normalize sums to get average biases
        accl_bias /= count;
        gyro_bias /= count;

        // Remove gravity from the z-axis accelerometer bias calculation
        accl_bias.z += (accl_bias.z > 0) ? -acclSensitivity : acclSensitivity;

        // Accumulate bias values for later averaging
        acclBias += accl_bias;
        gyroBias += gyro_bias;
        stream << trial+1 << ". accl_bias[x,y,z]=[" << accl_bias.x << ", " << accl_bias.y << ", " << accl_bias.z << ']' << endl;
        stream << trial+1 << ". gyro_bias[x,y,z]=[" << gyro_bias.x << ", " << gyro_bias.y << ", " << gyro_bias.z << ']' << endl;
    }

    acclBias /= countTrials;
    gyroBias /= countTrials;
    digitalWrite(LED_BUILTIN, LOW);
    stream << "Calibrated accl bias[x,y,z]=[" << acclBias.x << ", " << acclBias.y << ", " << acclBias.z << "]" << endl;
    stream << "Calibrated gyro bias[x,y,z]=[" << gyroBias.x << ", " << gyroBias.y << ", " << gyroBias.z << "]" << endl;

    SetAcclBias(acclBias);
    SetGyroBias(gyroBias);
}


void MPU9250::CalibrateFine(Print& stream)
{
    initMPU9250();

    Vector3F acclMax;
    Vector3F acclMin;
    Vector3F acclSum;
    auto n = 0L;
    auto t0 = millis();
    auto t1 = t0;
    auto endTime = t0 + 5000UL;

    accBiasFine = 0.0;
    acclThreshold = 0.0;

    while ((t1 = millis()) < endTime)
    {
        if ((Update() & 0x01) == 0) continue;

        auto a = GetAccel();

        a.z -= 1.0;         // Subtract gravity acceleration on z-axis
        acclSum += a;
        t0 = t1;

        if (++n == 1)       // On first pass initialize the min/max values
        {
            acclMax = a;
            acclMin = a;
        }
        else
        {
            if (a.x > acclMax.x) acclMax.x = a.x; // (acclMax.x + a.x) / 2;
            if (a.y > acclMax.y) acclMax.y = a.y; // (acclMax.x + a.x) / 2;
            if (a.z > acclMax.z) acclMax.z = a.z; // (acclMax.x + a.x) / 2;
            if (a.x < acclMin.x) acclMin.x = a.x; // (acclMax.x + a.x) / 2;
            if (a.y < acclMin.y) acclMin.y = a.y; // (acclMax.x + a.x) / 2;
            if (a.z < acclMin.z) acclMin.z = a.z; // (acclMax.x + a.x) / 2;
        }
    }

    accBiasFine = acclSum / n;
    acclThreshold.x = ((fabs(acclMax.x) + fabs(acclMin.x)) * 0.5) - accBiasFine.x;
    acclThreshold.y = ((fabs(acclMax.y) + fabs(acclMin.y)) * 0.5) - accBiasFine.y;
    acclThreshold.z = ((fabs(acclMax.z) + fabs(acclMin.z)) * 0.5) - accBiasFine.z;

    stream << "Calibrated accl fine bias[x,y,z]=[" << _FLOAT(accBiasFine.x,6)   << ", " << _FLOAT(accBiasFine.y,6)   << ", "   << _FLOAT(accBiasFine.z, 6) << "]" << endl;
    stream << "Calibrated accl threshold[x,y,z]=[" << _FLOAT(acclThreshold.x,6) << ", " << _FLOAT(acclThreshold.y,6) << ", " << _FLOAT(acclThreshold.z,6) << "]" << endl;
    digitalWrite(LED_BUILTIN, LOW);
}


int8_t MPU9250::Update()
{
    uint8_t result = 0;
    uint8_t rawData[7];  // Register data stored here
    
    if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
    {
        // Read the six raw data registers into data array
        readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);

        // Turn the MSB and LSB into a signed 16-bit value
        noInterrupts();
        acclRaw.x = ((int16_t)rawData[0] << 8) | rawData[1];
        acclRaw.y = ((int16_t)rawData[2] << 8) | rawData[3];
        acclRaw.z = ((int16_t)rawData[4] << 8) | rawData[5];
        interrupts();

        readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);

        // Turn the MSB and LSB into a signed 16-bit value
        noInterrupts();
        gyroRaw.x = ((int16_t)rawData[0] << 8) | rawData[1];
        gyroRaw.y = ((int16_t)rawData[2] << 8) | rawData[3];
        gyroRaw.z = ((int16_t)rawData[4] << 8) | rawData[5];
        interrupts();

        result |= 0x01;
    }

    // wait for magnetometer data ready bit to be set
    if (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01)
    {
        // Read the six raw data and ST2 registers sequentially into data array
        readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  

        auto st2 = rawData[6]; // Get ST2 register value

        // If magnetic sensor overflow not set then we have good data
        if (!(st2 & 0x08))
        {
            noInterrupts();
            magRaw.x = ((int16_t)rawData[1] << 8) | rawData[0];     // Turn the MSB and LSB into a signed 16-bit value
            magRaw.y = ((int16_t)rawData[3] << 8) | rawData[2];     // Data stored as little Endian
            magRaw.z = ((int16_t)rawData[5] << 8) | rawData[4];
            interrupts();

            result |= 0x02;
        }
    }

    if (result & 0x01)
    {
        auto aRes = GetAres();              // Accelerometer scaling factor
        auto gRes = GetGres()*DEG_TO_RAD;   // Gyroscope scaling factor combined with degress-to-radians conversion
        Vector3F a(acclRaw.x - acclBias.x, acclRaw.y - acclBias.y, acclRaw.z - acclBias.z);

        a *= aRes;
        a -= accBiasFine;
        a = lowPass(a, accl, 0.1);

        if (fabs(a.x) < acclThreshold.x) a.x = 0.0;
        if (fabs(a.y) < acclThreshold.y) a.y = 0.0;
        if (fabs(a.z) < acclThreshold.z) a.z = 0.0;

        noInterrupts();
        accl = a;
        gyro.x = gyroRaw.x * gRes;
        gyro.y = gyroRaw.y * gRes;
        gyro.z = gyroRaw.z * gRes;
        interrupts();

        // Run the accelerometer vector through a low-pass filter to smooth output
        // and minimize effect of noise
        lastAccl = lowPass(accl, lastAccl, 0.1);
    }

    if (result & 0x02)
    {
        auto mRes = GetMres();  // Magnetometer scaling factor

        noInterrupts();
        mag.x = (magRaw.y - magBias[1]) * mRes * magSens[1];   // x value = mag y-axis value
        mag.y = (magRaw.x - magBias[0]) * mRes * magSens[0];   // y value = mag x-axis value
        mag.z = -(magRaw.z - magBias[2]) * mRes * magSens[2];  // z value = mag -z-axis value
        interrupts();

        // Run the magnetometer vector through a low-pass filter to smooth output
        // and minimize effect of noise
        lastMag = lowPass(mag, lastMag, 0.1);

        // Update the compass heading
        UpdateCompassHeading();
    }

    return result;
}


//******************************************************************************
// Update tilt-compensated Compass heading
// Most accurate when device is under little or no acceleration
//******************************************************************************
float MPU9250::UpdateCompassHeading()
{
    // Used filtered magnetometer and accelerometer readings
    auto bp = lastMag;
    auto gp = lastAccl;

    // Calculate roll angle
    auto roll = atan2(gp.y, gp.z);          // Eq 13
    auto sr = sin(roll);
    auto cr = cos(roll);

    // de-rotate by roll angle
    auto by = bp.y*cr - bp.z*sr;            // Eq 19: y component

    bp.z = bp.y*sr + bp.z*cr;               // Bpz = py*sin(roll)+Bpz*cos(roll)
    gp.z = gp.y*sr + gp.z*cr;               // Eq 15 denominator

                                            // calculate current pitch angle
    auto pitch = atan(-gp.x / gp.z);        // Eq 15
    auto sp = sin(pitch);
    auto cp = cos(pitch);

    // de-rotate by pitch angle 
    auto bx = bp.x*cp + bp.z*sp;            // Eq 19: x component 
    //auto bz = -bp.x*sp + bp.z*cp;           // Eq 19: z component - not really needed

                                            // calculate current yaw angle
    auto yaw = atan2(-by, bx);              // Eq 22

                                            // Converts yaw angle to heading angle in range 0-360; where 0=North, 90=East, 180=South, 270=West
    heading = fmod((TWO_PI - yaw)*RAD_TO_DEG + IMU_MAG_CORRECTION, 360);

    return heading;
}


int16_t MPU9250::ReadTempData()
{
    uint8_t rawData[2];

    // Read the two raw data registers sequentially into data array
    readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);

    // Turn the MSB and LSB into a 16-bit value
    return ((int16_t)rawData[0] << 8) | rawData[1];
}


//==============================================================================
// Set of useful function to access acceleration. gyroscope, magnetometer, and 
// temperature data
//==============================================================================

float MPU9250::GetMres()
{
    float res = 0.0;

    switch (Mscale)
    {
    // Possible magnetometer scales (and their register bit settings) are:
    // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
        res = 10.*4912. / 8190.; // Proper scale to return milliGauss
        break;

    case MFS_16BITS:
        res = 10.*4912. / 32760.0; // Proper scale to return milliGauss
        break;
    }

    return res;
}


float MPU9250::GetGres()
{
    float res = 0.0;

    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
    switch (Gscale)
    {
    case GFS_250DPS:
        res = 250.0 / 32768.0;
        break;

    case GFS_500DPS:
        res = 500.0 / 32768.0;
        break;

    case GFS_1000DPS:
        res = 1000.0 / 32768.0;
        break;

    case GFS_2000DPS:
        res = 2000.0 / 32768.0;
        break;
    }

    return res;
}


float MPU9250::GetAres()
{
    float res = 0.0;

    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    switch (Ascale)
    {
    case AFS_2G:
        res = 2.0 / 32768.0;
        break;

    case AFS_4G:
        res = 4.0 / 32768.0;
        break;

    case AFS_8G:
        res = 8.0 / 32768.0;
        break;

    case AFS_16G:
        res = 16.0 / 32768.0;
        break;
    }

    return res;
}


void MPU9250::GetMagCalibration(float bias[3], float sensitivity[3])
{
    if (bias != NULL)
    {
        bias[0] = magBias[0];
        bias[1] = magBias[1];
        bias[2] = magBias[2];
    }

    if (sensitivity != NULL)
    {
        sensitivity[0] = magSens[0];
        sensitivity[1] = magSens[1];
        sensitivity[2] = magSens[2];
    }
}


void MPU9250::initMPU9250()
{
    // wake up device - Clear sleep mode bit (6), enable all sensors
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);
    delay(100); // Wait for all registers to reset

    // get stable time source
    // Auto select clock source to be PLL gyroscope reference if ready else
    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
    delay(200);

    // Configure Gyro and Thermometer
    // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
    // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
    // be higher than 1 / 0.0059 = 170 Hz
    // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
    // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
    writeByte(MPU9250_ADDRESS, CONFIG, 0x03);

    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    // Use a 200 Hz rate; a rate consistent with the filter update rate
    // determined inset in CONFIG above
    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
    // but these rates are reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting
    writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);

    // Set gyroscope full scale range
    // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
    uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value

    //c = c & ~0xE0;          // Clear self-test bits [7:5]
    c = c & ~0x03;          // Clear Fchoice bits [1:0]
    c = c & ~0x18;          // Clear GFS bits [4:3]
    c = c | Gscale << 3;    // Set full scale range for the gyro
    //c =| 0x00;              // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c); // Write new GYRO_CONFIG value to register

    // Set accelerometer full-scale range configuration
    c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
    //c = c & ~0xE0;          // Clear self-test bits [7:5]
    c = c & ~0x18;          // Clear AFS bits [4:3]
    c = c | Ascale << 3;    // Set full scale range for the accelerometer
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
    // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
    c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
    c = c & ~0x0F;          // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
    c = c | 0x03;           // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
    // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
    // can join the I2C bus and all can be controlled by the Arduino as master
    writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
    writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
    delay(100);
}


void MPU9250::initAK8963()
{
    uint8_t rawData[3];

    // Power down magnetometer to reset
    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00);
    delay(10);

    // Extract the factory calibration for each magnetometer axis
    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
    delay(10);

    // Read the x-, y-, and z-axis calibration values
    readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);

    // Compute sensitivity adjustment values
    magSens[0] = (float)(rawData[0] - 128) / 256. + 1.;
    magSens[1] = (float)(rawData[1] - 128) / 256. + 1.;
    magSens[2] = (float)(rawData[2] - 128) / 256. + 1.;
    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
    delay(10);

    // Configure the magnetometer for continuous read and highest resolution
    // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
    // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
    writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
    delay(10);
}


void MPU9250::SetAcclBias(Vector3F& bias)
{
    uint8_t data[12];

    // Construct the accelerometer biases for push to the hardware accelerometer 
    // bias registers. These registers contain factory trim values which must be
    // added to the calculated accelerometer biases; on boot up these registers
    // will hold non-zero values. In addition, bit 0 of the lower byte must be
    // preserved since it is used for temperature compensation calculations.
    // Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.
    int16_t accel_bias_reg[3] = { 0, 0, 0 };    // A place to hold the factory accelerometer trim biases
    int16_t mask_bit[3] = { 0, 0, 0 };          // Array to hold mask bit for each accelerometer bias axis

    readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
    accel_bias_reg[0] = (int32_t)(((int16_t)data[0] << 8) | data[1]);
    mask_bit[0] = accel_bias_reg[0] & 0x0001;

    readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[1] = (int32_t)(((int16_t)data[0] << 8) | data[1]);
    mask_bit[1] = accel_bias_reg[1] & 0x0001;

    readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[2] = (int32_t)(((int16_t)data[0] << 8) | data[1]);
    mask_bit[2] = accel_bias_reg[2] & 0x0001;

    //stream << "Accel_bias_reg[x,y,z]=[0x" << _HEX(accel_bias_reg[0]) << ", 0x" << _HEX(accel_bias_reg[1]) << ", 0x" << _HEX(accel_bias_reg[2]) << "]" << endl;

    // Shift out temperature compensation bit to get actual value
    accel_bias_reg[0] >>= 1;
    accel_bias_reg[1] >>= 1;
    accel_bias_reg[2] >>= 1;

    //stream << "Accel_bias_reg[x,y,z]=[" << accel_bias_reg[0] << ", " << accel_bias_reg[1] << ", " << accel_bias_reg[2] << "]" << endl;

    // Construct total accelerometer bias, including calculated average accelerometer
    // bias from above. Subtract calculated averaged accelerometer bias scaled to
    // 2048 LSB/g (16 g full scale).
    accel_bias_reg[0] -= (int16_t)(bias.x + 0.5f) / 8;
    accel_bias_reg[1] -= (int16_t)(bias.y + 0.5f) / 8;
    accel_bias_reg[2] -= (int16_t)(bias.z + 0.5f) / 8;

    //stream << "Accel_bias_reg[x,y,z]=[" << accel_bias_reg[0] << ", " << accel_bias_reg[1] << ", " << accel_bias_reg[2] << "]" << endl;

    // Prepare to write back to accelerometer bias registers
    // Reconstruct bias values to include temperature compensation bit
    accel_bias_reg[0] = (accel_bias_reg[0] << 1) | mask_bit[0];
    accel_bias_reg[1] = (accel_bias_reg[1] << 1) | mask_bit[1];
    accel_bias_reg[2] = (accel_bias_reg[2] << 1) | mask_bit[2];

    //stream << "Accel_bias_reg[x,y,z]=[0x" << _HEX(accel_bias_reg[0]) << ", 0x" << _HEX(accel_bias_reg[1]) << ", 0x" << _HEX(accel_bias_reg[2]) << "]" << endl;

    // Write the accel bias values
    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0]) & 0xFF;
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1]) & 0xFF;
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2]) & 0xFF;

    // Push accelerometer biases to hardware registers.
    // Do this despite the fact that it doesn't aapear to be working on the MPU-9250
    // (subsequent acceleration readings still show significant bias offset).
    // Are we handling the temperature correction bit properly?
    //writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
    //writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
    //writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
    //writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
    //writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
    //writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
}


void MPU9250::SetGyroBias(Vector3F& bias)
{
    // Construct the gyro biases for push to the hardware gyro bias registers, 
    // which are reset to zero upon device startup. 
    // Divide bias values by 4 to get 32.9 LSB per deg/s to conform to expected
    // bias input format. Biases are added by MPU9250, so change sign on gyro
    // biases. Round-up bias values and do all artimetic in floating-point to
    // minimize rounding errors before converting to integer.
    int16_t biasX = -(bias.x + 0.5) / 4;
    int16_t biasY = -(bias.y + 0.5) / 4;
    int16_t biasZ = -(bias.z + 0.5) / 4;
    uint8_t data[12];

    data[0] = (biasX  >> 8) & 0xFF;
    data[1] = (biasX) & 0xFF;
    data[2] = (biasY >> 8) & 0xFF;
    data[3] = (biasY) & 0xFF;
    data[4] = (biasZ >> 8) & 0xFF;
    data[5] = (biasZ) & 0xFF;

    // Push gyro biases to hardware registers
    writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
    writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
    writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
    writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
    writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
    writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);
}


// Wire.h read and write protocols
void MPU9250::writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
    Wire.beginTransmission(address);  // Initialize the Tx buffer
    Wire.write(subAddress);           // Put slave register address in Tx buffer
    Wire.write(data);                 // Put data in Tx buffer
    Wire.endTransmission();           // Send the Tx buffer
}


uint8_t MPU9250::readByte(uint8_t address, uint8_t subAddress)
{
    uint8_t data; // `data` will store the register data
    
    Wire.beginTransmission(address);         // Initialize the Tx buffer
    Wire.write(subAddress);                  // Put slave register address in Tx buffer
    Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
    Wire.requestFrom(address, (uint8_t)1);   // Read one byte from slave register address
    data = Wire.read();                      // Fill Rx buffer with result

    return data;                             // Return data read from slave register
}


void MPU9250::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
    Wire.beginTransmission(address);   // Initialize the Tx buffer
    Wire.write(subAddress);            // Put slave register address in Tx buffer
    Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive

    uint8_t i = 0;

    Wire.requestFrom(address, count);  // Read bytes from slave register address

    while (Wire.available())
    {
        dest[i++] = Wire.read();       // Put read results in the Rx buffer
    }
}


//******************************************************************************
/// time smoothing constant for low-pass filter
/// 0 ≤ alpha ≤ 1 ; a smaller value basically means more smoothing
/// @see http://en.wikipedia.org/wiki/Low-pass_filter#Discrete-time_realization
/// @see http://en.wikipedia.org/wiki/Low-pass_filter#Algorithmic_implementation
//******************************************************************************
float lowPass(float newValue, float oldValue, float alpha)
{
    auto output = oldValue + alpha * (newValue - oldValue);

    return output;
}


//******************************************************************************
/// time smoothing constant for low-pass filter
/// 0 ≤ alpha ≤ 1 ; a smaller value basically means more smoothing
/// @see http://en.wikipedia.org/wiki/Low-pass_filter#Discrete-time_realization
/// @see http://en.wikipedia.org/wiki/Low-pass_filter#Algorithmic_implementation
//******************************************************************************
Vector3F lowPass(Vector3F& newValue, Vector3F& oldValue, float alpha)
{
    auto output = Vector3F(oldValue);

    output.x += alpha * (newValue.x - oldValue.x);
    output.y += alpha * (newValue.y - oldValue.y);
    output.z += alpha * (newValue.z - oldValue.z);

    return output;
}


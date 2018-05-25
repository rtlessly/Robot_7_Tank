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
#include "MPU9250.h"
#include "MPU9250RegisterMap.h"
#include "quaternionFilters.h"


#ifndef ConsoleStream
#define ConsoleStream SerialUSB
#endif

#ifndef DataStream
#define DataStream SerialUSB
#endif

#ifndef TelemetryStream
#define TelemetryStream SerialUSB
#endif


//// Forward declarations
//void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
//uint8_t readByte(uint8_t address, uint8_t subAddress);
//void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);


//uint8_t Ascale = AFS_2G;                    // Specify acceleromter scale setting
//uint8_t Gscale = GFS_250DPS;                // Specify gyroscope scale setting
//uint8_t Mscale = MFS_16BITS;                // Specify magentometer scale setting - Choose either 14-bit or 16-bit resolution
//uint8_t Mmode = 0x02;                       // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
//
//float   gyroBias[3] = { 0, 0, 0 };          // Bias corrections for gyro
//float   accelBias[3] = { 0, 0, 0 };         // Bias corrections for accelerometer
//float   magbias[3] = { 0, 0, 0 };           // Magnetometer bias calibration (milliGauss)


MPU9250::MPU9250()
{
    Ascale = AFS_2G;            // Specify acceleromter scale setting
    Gscale = GFS_250DPS;        // Specify gyroscope scale setting
    Mscale = MFS_16BITS;        // Specify magentometer scale setting - Choose either 14-bit or 16-bit resolution
    Mmode = 0x02;               // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
    
    // Set the mag bias values in milliGauss. These are empirically determined for each sensor
    magBias[0] = MAG_BIASX; // +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
    magBias[1] = MAG_BIASY; // +120.;  // User environmental y-axis correction in milliGauss
    magBias[2] = MAG_BIASZ; // +125.;  // User environmental z-axis correction in milliGauss

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


void MPU9250::Initialize(Print& stream)
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
    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
    int32_t gyro_bias[3] = { 0, 0, 0 };
    int32_t accel_bias[3] = { 0, 0, 0 };

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

    int32_t gyrosensitivity = 131;                  // = 131 LSB/degrees/sec
    int32_t accelsensitivity = 16384;               // = 16384 LSB/g
    int32_t count = 0;

    for (int iter = 0; iter < 25; iter++)
    {
        // Configure FIFO to capture accelerometer and gyro data for bias calculation
        writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);    // Enable FIFO
        writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);      // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
        delay(40);                                      // accumulate 40 samples in 40 milliseconds = 480 bytes

        // At end of sample accumulation, turn off FIFO sensor read
        writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);            // Disable gyro and accelerometer sensors for FIFO
        readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count

        int fifo_count = ((uint16_t)data[0] << 8) | data[1];
        int packet_count = fifo_count / 12;// How many sets of full gyro and accelerometer data for averaging

        for (int ii = 0; ii < packet_count; ii++)
        {
            int16_t accel_temp[3] = { 0, 0, 0 };
            int16_t gyro_temp[3] = { 0, 0, 0 };

            // read data for averaging
            readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]);

            // Form signed 16-bit integer for each sample in FIFO
            accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]);
            accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
            accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
            gyro_temp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
            gyro_temp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
            gyro_temp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

            //stream << "Accel[x,y,z]=[" << accel_temp[0] << ", " << accel_temp[1] << ", " << accel_temp[2] << "]" << endl;

            // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
            accel_bias[0] += (int32_t)accel_temp[0];
            accel_bias[1] += (int32_t)accel_temp[1];
            accel_bias[2] += (int32_t)accel_temp[2];
            gyro_bias[0] += (int32_t)gyro_temp[0];
            gyro_bias[1] += (int32_t)gyro_temp[1];
            gyro_bias[2] += (int32_t)gyro_temp[2];
            count++;
        }
    }

    // Normalize sums to get average count biases
    accel_bias[0] /= count;
    accel_bias[1] /= count;
    accel_bias[2] /= count;
    gyro_bias[0] /= count;
    gyro_bias[1] /= count;
    gyro_bias[2] /= count;

    // Remove gravity from the z-axis accelerometer bias calculation
    if (accel_bias[2] > 0L)
    {
        accel_bias[2] -= accelsensitivity;
    }
    else
    {
        accel_bias[2] += accelsensitivity;
    }

    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF;  // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0] / 4) & 0xFF;       // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
    data[3] = (-gyro_bias[1] / 4) & 0xFF;
    data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
    data[5] = (-gyro_bias[2] / 4) & 0xFF;

    // Push gyro biases to hardware registers
    writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
    writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
    writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
    writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
    writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
    writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);

    // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.
    
    int16_t accel_bias_reg[3] = { 0, 0, 0 }; // A place to hold the factory accelerometer trim biases
    int16_t mask_bit[3] = { 0, 0, 0 }; // Array to hold mask bit for each accelerometer bias axis

    readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
    accel_bias_reg[0] = (int32_t)(((int16_t)data[0] << 8) | data[1]);
    mask_bit[0] = accel_bias_reg[0] & 0x0001;
    
    readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[1] = (int32_t)(((int16_t)data[0] << 8) | data[1]);
    mask_bit[1] = accel_bias_reg[1] & 0x0001;
    
    readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[2] = (int32_t)(((int16_t)data[0] << 8) | data[1]);
    mask_bit[2] = accel_bias_reg[2] & 0x0001;

    stream << "Accel_bias_reg[x,y,z]=[" << accel_bias_reg[0] << ", " << accel_bias_reg[1] << ", " << accel_bias_reg[2] << "]" << endl;
    stream << "Accel_bias_reg[x,y,z]=[" << _HEX(accel_bias_reg[0]) << ", " << _HEX(accel_bias_reg[1]) << ", " << _HEX(accel_bias_reg[2]) << "]" << endl;

    // Shift out temperature compensation bit to get actual value
    accel_bias_reg[0] >>= 1;
    accel_bias_reg[1] >>= 1;
    accel_bias_reg[2] >>= 1;

    stream << "Accel_bias_reg[x,y,z]=[" << accel_bias_reg[0] << ", " << accel_bias_reg[1] << ", " << accel_bias_reg[2] << "]" << endl;

    // Construct total accelerometer bias, including calculated average accelerometer bias from above
    // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
    accel_bias_reg[0] -= (int16_t)(accel_bias[0] / 8);
    accel_bias_reg[1] -= (int16_t)(accel_bias[1] / 8);
    accel_bias_reg[2] -= (int16_t)(accel_bias[2] / 8);

    stream << "Accel_bias_reg[x,y,z]=[" << accel_bias_reg[0] << ", " << accel_bias_reg[1] << ", " << accel_bias_reg[2] << "]" << endl;

    // Prepare to write back to accelerometer bias registers
    // Reconstruct bias values to include temperature compensation bit
    accel_bias_reg[0] = (accel_bias_reg[0] << 1) | mask_bit[0];
    accel_bias_reg[1] = (accel_bias_reg[1] << 1) | mask_bit[1];
    accel_bias_reg[2] = (accel_bias_reg[2] << 1) | mask_bit[2];

    stream << "Accel_bias_reg[x,y,z]=[" << accel_bias_reg[0] << ", " << accel_bias_reg[1] << ", " << accel_bias_reg[2] << "]" << endl;
    stream << "Accel_bias_reg[x,y,z]=[" << _HEX(accel_bias_reg[0]) << ", " << _HEX(accel_bias_reg[1]) << ", " << _HEX(accel_bias_reg[2]) << "]" << endl;

    // Write the accel bias values
    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0]) & 0xFF;
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1]) & 0xFF;
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2]) & 0xFF;

    // Push accelerometer biases to hardware registers
    writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
    writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
    writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
    writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
    writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
    writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);

    // Output scaled gyro biases for display in the main program
    gyroBias[0] = (float)gyro_bias[0] / (float)gyrosensitivity;
    gyroBias[1] = (float)gyro_bias[1] / (float)gyrosensitivity;
    gyroBias[2] = (float)gyro_bias[2] / (float)gyrosensitivity;

    // Output scaled accelerometer biases for display in the main program
    accelBias[0] = (float)accel_bias[0] / (float)accelsensitivity;
    accelBias[1] = (float)accel_bias[1] / (float)accelsensitivity;
    accelBias[2] = (float)accel_bias[2] / (float)accelsensitivity;

    stream << "Gyro bias X: raw=" << gyro_bias[0] << ", scaled=" << _FLOAT(gyroBias[0], 3) << endl;
    stream << "Gyro bias Y: raw=" << gyro_bias[1] << ", scaled=" << _FLOAT(gyroBias[1], 3) << endl;
    stream << "Gyro bias Z: raw=" << gyro_bias[2] << ", scaled=" << _FLOAT(gyroBias[2], 3) << endl;
    stream << "Accel bias X: raw=" << accel_bias[0] << ", scaled=" << _FLOAT(accelBias[0], 3) << endl;
    stream << "Accel bias Y: raw=" << accel_bias[1] << ", scaled=" << _FLOAT(accelBias[1], 3) << endl;
    stream << "Accel bias Z: raw=" << accel_bias[2] << ", scaled=" << _FLOAT(accelBias[2], 3) << endl;
}


bool MPU9250::DataReady()
{
    // If interrupt bit goes high, all data registers have new data ready
    return readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01;
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
        accelRaw.X = ((int16_t)rawData[0] << 8) | rawData[1];
        accelRaw.Y = ((int16_t)rawData[2] << 8) | rawData[3];
        accelRaw.Z = ((int16_t)rawData[4] << 8) | rawData[5];
        interrupts();

        readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);

        // Turn the MSB and LSB into a signed 16-bit value
        noInterrupts();
        // Turn the MSB and LSB into a signed 16-bit value
        gyroRaw.X = ((int16_t)rawData[0] << 8) | rawData[1];
        gyroRaw.Y = ((int16_t)rawData[2] << 8) | rawData[3];
        gyroRaw.Z = ((int16_t)rawData[4] << 8) | rawData[5];
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
            magRaw.X = ((int16_t)rawData[1] << 8) | rawData[0];  // Turn the MSB and LSB into a signed 16-bit value
            magRaw.Y = ((int16_t)rawData[3] << 8) | rawData[2];  // Data stored as little Endian
            magRaw.Z = ((int16_t)rawData[5] << 8) | rawData[4];
            interrupts();
            result |= 0x02;
        }
    }

    return result;
}


void MPU9250::GetAccelRaw(RawData& accelData)
{
    accelData.X = accelRaw.X;
    accelData.Y = accelRaw.Y;
    accelData.Z = accelRaw.Z;
}


void MPU9250::GetAccel(ScaledData& accelData)
{
    auto aRes = GetAres();  // Acceleration scaling factor

    // Convert acceleration values to g's
    accelData.X = (float)accelRaw.X * aRes; // - accelBias[0];
    accelData.Y = (float)accelRaw.Y * aRes; // - accelBias[1];
    accelData.Z = (float)accelRaw.Z * aRes; // - accelBias[2];
}


void MPU9250::GetGyroRaw(RawData& gyroData)
{
    gyroData.X = gyroRaw.X;
    gyroData.Y = gyroRaw.Y;
    gyroData.Z = gyroRaw.Z;
}


void MPU9250::GetGyro(ScaledData& gyroData)
{
    auto gRes = GetGres();  // Gyroscope scaling factor

    // Convert gyro valued to degrees per second
    gyroData.X = (float)gyroRaw.X * gRes;
    gyroData.Y = (float)gyroRaw.Y * gRes;
    gyroData.Z = (float)gyroRaw.Z * gRes;
}


void MPU9250::GetMagRaw(RawData& magData)
{
    magData.X = magRaw.X;
    magData.Y = magRaw.Y;
    magData.Z = magRaw.Z;
}


void MPU9250::GetMag(ScaledData& magData)
{
    auto mRes = GetMres();  // Magnetometer scaling factor

    // Convert magnetometer values to milliGauss
    magData.X = (magRaw.X - magBias[0]) * magSens[0] * mRes;
    magData.Y = (magRaw.Y - magBias[1]) * magSens[1] * mRes;
    magData.Z = (magRaw.Z - magBias[2]) * magSens[2] * mRes;
}


int16_t MPU9250::ReadTempData()
{
    uint8_t rawData[2];

    readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array

    return ((int16_t)rawData[0] << 8) | rawData[1];  // Turn the MSB and LSB into a 16-bit value
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


void MPU9250::GetAccelCalibration(float bias[3])
{
    if (bias != NULL)
    {
        bias[0] = accelBias[0];
        bias[1] = accelBias[1];
        bias[2] = accelBias[2];
    }
}


void MPU9250::GetGyroCalibration(float bias[3])
{
    if (bias != NULL)
    {
        bias[0] = gyroBias[0];
        bias[1] = gyroBias[1];
        bias[2] = gyroBias[2];
    }
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

    //c = c & ~0xE0; // Clear self-test bits [7:5]
    c = c & ~0x03; // Clear Fchoice bits [1:0]
    c = c & ~0x18; // Clear GFS bits [4:3]
    c = c | Gscale << 3; // Set full scale range for the gyro
    //c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c); // Write new GYRO_CONFIG value to register

    // Set accelerometer full-scale range configuration
    c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
    //c = c & ~0xE0; // Clear self-test bits [7:5]
    c = c & ~0x18;  // Clear AFS bits [4:3]
    c = c | Ascale << 3; // Set full scale range for the accelerometer
    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

    // Set accelerometer sample rate configuration
    // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
    // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
    c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
    c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
    c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
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









//bool RazorIMU_TestConnection()
//{
//    // Read the WHO_AM_I register of MPU-9250, this is a good test of communication
//    byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
//
//    ConsoleStream << "MPU9250: I AM " << _HEX(c) << "; I should be " << _HEX(0x71) << endl;
//
//    if (c != 0x71) // if WHO_AM_I returned incorrect value then abort
//    {
//        ConsoleStream << "Could not connect to MPU9250 - incorrect chip ID read: " << _HEX(c) << endl;
//        return false;
//    }
//
//    // Read the WHO_AM_I register of AK8963, this is a good test of communication
//    byte d = readByte(AK8963_ADDRESS, AK8963_WHO_AM_I);
//
//    ConsoleStream << endl;
//    ConsoleStream << "AK8963: I AM " << _HEX(d) << "; I should be " << _HEX(0x48) << endl;
//
//    if (d != 0x48) // if WHO_AM_I returned incorrect value then abort
//    {
//        ConsoleStream << "Could not connect to AK8963 - incorrect chip ID read: " << _HEX(d) << endl;
//        return false;
//    }
//
//    return true;
//}
//
//
//void RazorIMU_Initialize(float* magCalibration)
//{
//    // Initialize device for active mode read of accelerometer, gyroscope, and temperature
//    initMPU9250();
//    ConsoleStream << endl;
//    ConsoleStream << "MPU-9250 initialized for active data mode...." << endl;
//
//    // Set the mag bias values in milliGauss. These are empirically determined for each sensor
//    magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
//    magbias[1] = +120.;  // User environmental y-axis correction in milliGauss
//    magbias[2] = +125.;  // User environmental z-axis correction in milliGauss
//
//                         // Get magnetometer calibration from AK8963 ROM
//                         // Initialize device for active mode read of magnetometer
//    initAK8963(magCalibration);
//
//    ConsoleStream << endl;
//    ConsoleStream << "AK8963 initialized for active data mode...." << endl;
//}
//
//
//// Calibrates gyro and accelerometer after device initialization.
//// This calculates the average of the at-rest readings and then loads the
//// resulting offsets into accelerometer and gyro bias registers.
//void RazorIMU_Calibrate()
//{
//    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
//    int32_t gyro_bias[3] = { 0, 0, 0 };
//    int32_t accel_bias[3] = { 0, 0, 0 };
//    //uint16_t ii;
//
//    // reset device
//    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
//    delay(100);
//
//    // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
//    // else use the internal oscillator, bits 2:0 = 001
//    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
//    writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
//    delay(200);
//
//    // Configure device for bias calculation
//    writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
//    writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
//    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
//    writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
//    writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
//    writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
//    delay(15);
//
//    // Configure MPU6050 gyro and accelerometer for bias calculation
//    writeByte(MPU9250_ADDRESS, CONFIG, 0x01);       // Set low-pass filter to 188 Hz
//    writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);   // Set sample rate to 1 kHz
//    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
//    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
//
//    int32_t gyrosensitivity = 131;    // = 131 LSB/degrees/sec
//    int32_t accelsensitivity = 16384;  // = 16384 LSB/g
//    int32_t count = 0;
//
//    for (int iter = 0; iter < 25; iter++)
//    {
//        // Configure FIFO to capture accelerometer and gyro data for bias calculation
//        writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
//        writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
//        delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes
//
//                   // At end of sample accumulation, turn off FIFO sensor read
//        writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);            // Disable gyro and accelerometer sensors for FIFO
//        readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
//
//        int fifo_count = ((uint16_t)data[0] << 8) | data[1];
//        int packet_count = fifo_count / 12;// How many sets of full gyro and accelerometer data for averaging
//
//        for (int ii = 0; ii < packet_count; ii++)
//        {
//            int16_t accel_temp[3] = { 0, 0, 0 };
//            int16_t gyro_temp[3] = { 0, 0, 0 };
//
//            // read data for averaging
//            readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]);
//
//            // Form signed 16-bit integer for each sample in FIFO
//            accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]);
//            accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
//            accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
//            gyro_temp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
//            gyro_temp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
//            gyro_temp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);
//
//            //ConsoleStream << "Accel[x,y,z]=[" << accel_temp[0] << ", " << accel_temp[1] << ", " << accel_temp[2] << "]" << endl;
//
//            // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
//            accel_bias[0] += (int32_t)accel_temp[0];
//            accel_bias[1] += (int32_t)accel_temp[1];
//            accel_bias[2] += (int32_t)accel_temp[2];
//            gyro_bias[0] += (int32_t)gyro_temp[0];
//            gyro_bias[1] += (int32_t)gyro_temp[1];
//            gyro_bias[2] += (int32_t)gyro_temp[2];
//            count++;
//        }
//    }
//
//    // Normalize sums to get average count biases
//    accel_bias[0] /= count;
//    accel_bias[1] /= count;
//    accel_bias[2] /= count;
//    gyro_bias[0] /= count;
//    gyro_bias[1] /= count;
//    gyro_bias[2] /= count;
//
//    // Remove gravity from the z-axis accelerometer bias calculation
//    if (accel_bias[2] > 0L)
//    {
//        accel_bias[2] -= accelsensitivity;
//    }
//    else
//    {
//        accel_bias[2] += accelsensitivity;
//    }
//
//    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
//    data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
//    data[1] = (-gyro_bias[0] / 4) & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
//    data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
//    data[3] = (-gyro_bias[1] / 4) & 0xFF;
//    data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
//    data[5] = (-gyro_bias[2] / 4) & 0xFF;
//
//    // Push gyro biases to hardware registers
//    writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
//    writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
//    writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
//    writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
//    writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
//    writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);
//
//    // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
//    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
//    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
//    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
//    // the accelerometer biases calculated above must be divided by 8.
//
//    int16_t accel_bias_reg[3] = { 0, 0, 0 }; // A place to hold the factory accelerometer trim biases
//    int16_t mask_bit[3] = { 0, 0, 0 }; // Array to hold mask bit for each accelerometer bias axis
//
//    readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
//    accel_bias_reg[0] = (int32_t)(((int16_t)data[0] << 8) | data[1]);
//    mask_bit[0] = accel_bias_reg[0] & 0x0001;
//    readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
//    accel_bias_reg[1] = (int32_t)(((int16_t)data[0] << 8) | data[1]);
//    mask_bit[1] = accel_bias_reg[1] & 0x0001;
//    readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
//    accel_bias_reg[2] = (int32_t)(((int16_t)data[0] << 8) | data[1]);
//    mask_bit[2] = accel_bias_reg[2] & 0x0001;
//
//    ConsoleStream << "Accel_bias_reg[x,y,z]=[" << accel_bias_reg[0] << ", " << accel_bias_reg[1] << ", " << accel_bias_reg[2] << "]" << endl;
//    ConsoleStream << "Accel_bias_reg[x,y,z]=[" << _HEX(accel_bias_reg[0]) << ", " << _HEX(accel_bias_reg[1]) << ", " << _HEX(accel_bias_reg[2]) << "]" << endl;
//
//    // Shift out temperature compensation bit to get actual value
//    accel_bias_reg[0] >>= 1;
//    accel_bias_reg[1] >>= 1;
//    accel_bias_reg[2] >>= 1;
//
//    ConsoleStream << "Accel_bias_reg[x,y,z]=[" << accel_bias_reg[0] << ", " << accel_bias_reg[1] << ", " << accel_bias_reg[2] << "]" << endl;
//
//    // Construct total accelerometer bias, including calculated average accelerometer bias from above
//    // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
//    accel_bias_reg[0] -= (int16_t)(accel_bias[0] / 8);
//    accel_bias_reg[1] -= (int16_t)(accel_bias[1] / 8);
//    accel_bias_reg[2] -= (int16_t)(accel_bias[2] / 8);
//
//    ConsoleStream << "Accel_bias_reg[x,y,z]=[" << accel_bias_reg[0] << ", " << accel_bias_reg[1] << ", " << accel_bias_reg[2] << "]" << endl;
//
//    // Prepare to write back to accelerometer bias registers
//    // Reconstruct bias values to include temperature compensation bit
//    accel_bias_reg[0] = (accel_bias_reg[0] << 1) | mask_bit[0];
//    accel_bias_reg[1] = (accel_bias_reg[1] << 1) | mask_bit[1];
//    accel_bias_reg[2] = (accel_bias_reg[2] << 1) | mask_bit[2];
//
//    ConsoleStream << "Accel_bias_reg[x,y,z]=[" << accel_bias_reg[0] << ", " << accel_bias_reg[1] << ", " << accel_bias_reg[2] << "]" << endl;
//    ConsoleStream << "Accel_bias_reg[x,y,z]=[" << _HEX(accel_bias_reg[0]) << ", " << _HEX(accel_bias_reg[1]) << ", " << _HEX(accel_bias_reg[2]) << "]" << endl;
//
//    // Write the accel bias values
//    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
//    data[1] = (accel_bias_reg[0]) & 0xFF;
//    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
//    data[3] = (accel_bias_reg[1]) & 0xFF;
//    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
//    data[5] = (accel_bias_reg[2]) & 0xFF;
//
//    // Push accelerometer biases to hardware registers
//    writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
//    writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
//    writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
//    writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
//    writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
//    writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
//
//    // Output scaled gyro biases for display in the main program
//    gyroBias[0] = (float)gyro_bias[0] / (float)gyrosensitivity;
//    gyroBias[1] = (float)gyro_bias[1] / (float)gyrosensitivity;
//    gyroBias[2] = (float)gyro_bias[2] / (float)gyrosensitivity;
//
//    // Output scaled accelerometer biases for display in the main program
//    accelBias[0] = (float)accel_bias[0] / (float)accelsensitivity;
//    accelBias[1] = (float)accel_bias[1] / (float)accelsensitivity;
//    accelBias[2] = (float)accel_bias[2] / (float)accelsensitivity;
//
//    ConsoleStream << "Gyro bias X: raw=" << gyro_bias[0] << ", scaled=" << _FLOAT(gyroBias[0], 3) << endl;
//    ConsoleStream << "Gyro bias Y: raw=" << gyro_bias[1] << ", scaled=" << _FLOAT(gyroBias[1], 3) << endl;
//    ConsoleStream << "Gyro bias Z: raw=" << gyro_bias[2] << ", scaled=" << _FLOAT(gyroBias[2], 3) << endl;
//    ConsoleStream << "Accel bias X: raw=" << accel_bias[0] << ", scaled=" << _FLOAT(accelBias[0], 3) << endl;
//    ConsoleStream << "Accel bias Y: raw=" << accel_bias[1] << ", scaled=" << _FLOAT(accelBias[1], 3) << endl;
//    ConsoleStream << "Accel bias Z: raw=" << accel_bias[2] << ", scaled=" << _FLOAT(accelBias[2], 3) << endl;
//}
//
//
//// Accelerometer and gyroscope self test; check calibration wrt factory settings
//// Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
//void RazorIMU_SelfTest(float* pResuts)
//{
//    uint8_t rawData[6] = { 0, 0, 0, 0, 0, 0 };
//    uint8_t selfTest[6];
//    int32_t gAvg[3] = { 0 }, aAvg[3] = { 0 }, aSTAvg[3] = { 0 }, gSTAvg[3] = { 0 };
//    float factoryTrim[6];
//    uint8_t FS = 0;
//
//    writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
//    writeByte(MPU9250_ADDRESS, CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
//    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, FS << 3);  // Set full scale range for the gyro to 250 dps
//    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
//    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, FS << 3); // Set full scale range for the accelerometer to 2 g
//
//                                                       // get average current values of gyro and accelerometer
//    for (int ii = 0; ii < 200; ii++)
//    {
//        readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers into data array
//        aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
//        aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
//        aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
//
//        readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers sequentially into data array
//        gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
//        gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
//        gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
//    }
//
//    // Get average of 200 values and store as average current readings
//    for (int ii = 0; ii < 3; ii++)
//    {
//        aAvg[ii] /= 200;
//        gAvg[ii] /= 200;
//    }
//
//    // Configure the accelerometer for self-test
//    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
//    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
//    delay(25);  // Delay a while to let the device stabilize
//
//                // get average self-test values of gyro and accelerometer
//    for (int ii = 0; ii < 200; ii++)
//    {
//        readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
//        aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value
//        aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
//        aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
//
//        readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
//        gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value
//        gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
//        gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
//    }
//
//    // Get average of 200 values and store as average self-test readings
//    for (int ii = 0; ii < 3; ii++)
//    {
//        aSTAvg[ii] /= 200;
//        gSTAvg[ii] /= 200;
//    }
//
//    // Configure the gyro and accelerometer for normal operation
//    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);
//    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);
//    delay(25);  // Delay a while to let the device stabilize
//
//                // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
//    selfTest[0] = readByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL); // X-axis accel self-test results
//    selfTest[1] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
//    selfTest[2] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
//    selfTest[3] = readByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO);  // X-axis gyro self-test results
//    selfTest[4] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
//    selfTest[5] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results
//
//                                                                // Retrieve factory self-test value from self-test code reads
//    factoryTrim[0] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[0] - 1.0))); // FT[Xa] factory trim calculation
//    factoryTrim[1] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[1] - 1.0))); // FT[Ya] factory trim calculation
//    factoryTrim[2] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[2] - 1.0))); // FT[Za] factory trim calculation
//    factoryTrim[3] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[3] - 1.0))); // FT[Xg] factory trim calculation
//    factoryTrim[4] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[4] - 1.0))); // FT[Yg] factory trim calculation
//    factoryTrim[5] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[5] - 1.0))); // FT[Zg] factory trim calculation
//
//                                                                                      // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
//                                                                                      // To get percent, must multiply by 100
//    for (int i = 0; i < 3; i++)
//    {
//        pResuts[i] = 100.0*((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i] - 100.;   // Report percent differences
//        pResuts[i + 3] = 100.0*((float)(gSTAvg[i] - gAvg[i])) / factoryTrim[i + 3] - 100.; // Report percent differences
//    }
//}
//
//
//bool RazorIMU_Ready()
//{
//    // If intPin goes high, all data registers have new data
//    // On interrupt, check if data ready interrupt
//    return readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01;
//}
//
//
//void readAccelData(int16_t * destination)
//{
//    uint8_t rawData[6];  // x/y/z accel register data stored here
//
//    readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);   // Read the six raw data registers into data array
//    destination[0] = ((int16_t)rawData[0] << 8) | rawData[1];  // Turn the MSB and LSB into a signed 16-bit value
//    destination[1] = ((int16_t)rawData[2] << 8) | rawData[3];
//    destination[2] = ((int16_t)rawData[4] << 8) | rawData[5];
//}
//
//
//void readGyroData(int16_t * destination)
//{
//    uint8_t rawData[6];  // x/y/z gyro register data stored here
//
//    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);    // Read the six raw data registers sequentially into data array
//    destination[0] = ((int16_t)rawData[0] << 8) | rawData[1];  // Turn the MSB and LSB into a signed 16-bit value
//    destination[1] = ((int16_t)rawData[2] << 8) | rawData[3];
//    destination[2] = ((int16_t)rawData[4] << 8) | rawData[5];
//}
//
//
//void readMagData(int16_t * destination)
//{
//    uint8_t rawData[7];  // x/y/z mag register data, ST2 register stored last, must read ST2 at end of data acquisition
//
//                         // wait for magnetometer data ready bit to be set
//    if (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01)
//    {
//        readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
//
//        uint8_t c = rawData[6]; // End data read by reading ST2 register
//
//                                // If magnetic sensor overflow not set then we have good data
//        if (!(c & 0x08))
//        {
//            destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];  // Turn the MSB and LSB into a signed 16-bit value
//            destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];  // Data stored as little Endian
//            destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
//        }
//    }
//}
//
//
//int16_t readTempData()
//{
//    uint8_t rawData[2];
//
//    readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
//
//    return ((int16_t)rawData[0] << 8) | rawData[1];  // Turn the MSB and LSB into a 16-bit value
//}
//
//
//float RazorIMU_GetMres()
//{
//    float res = 0.0;
//
//    switch (Mscale)
//    {
//        // Possible magnetometer scales (and their register bit settings) are:
//        // 14 bit resolution (0) and 16 bit resolution (1)
//    case MFS_14BITS:
//        res = 10.*4912. / 8190.; // Proper scale to return milliGauss
//        break;
//
//    case MFS_16BITS:
//        res = 10.*4912. / 32760.0; // Proper scale to return milliGauss
//        break;
//    }
//
//    return res;
//}
//
//
//float RazorIMU_GetGres()
//{
//    float res = 0.0;
//
//    // Possible gyro scales (and their register bit settings) are:
//    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
//    switch (Gscale)
//    {
//    case GFS_250DPS:
//        res = 250.0 / 32768.0;
//        break;
//
//    case GFS_500DPS:
//        res = 500.0 / 32768.0;
//        break;
//
//    case GFS_1000DPS:
//        res = 1000.0 / 32768.0;
//        break;
//
//    case GFS_2000DPS:
//        res = 2000.0 / 32768.0;
//        break;
//    }
//
//    return res;
//}
//
//
//float RazorIMU_GetAres()
//{
//    float res = 0.0;
//
//    // Possible accelerometer scales (and their register bit settings) are:
//    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
//    switch (Ascale)
//    {
//    case AFS_2G:
//        res = 2.0 / 32768.0;
//        break;
//
//    case AFS_4G:
//        res = 4.0 / 32768.0;
//        break;
//
//    case AFS_8G:
//        res = 8.0 / 32768.0;
//        break;
//
//    case AFS_16G:
//        res = 16.0 / 32768.0;
//        break;
//    }
//
//    return res;
//}
//
//
//void initMPU9250()
//{
//    // wake up device - Clear sleep mode bit (6), enable all sensors
//    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);
//    delay(100); // Wait for all registers to reset
//
//                // get stable time source
//                // Auto select clock source to be PLL gyroscope reference if ready else
//    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
//    delay(200);
//
//    // Configure Gyro and Thermometer
//    // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
//    // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
//    // be higher than 1 / 0.0059 = 170 Hz
//    // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
//    // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
//    writeByte(MPU9250_ADDRESS, CONFIG, 0x03);
//
//    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
//    // Use a 200 Hz rate; a rate consistent with the filter update rate
//    // determined inset in CONFIG above
//    // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
//    // but these rates are reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting
//    writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);
//
//    // Set gyroscope full scale range
//    // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
//    uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
//
//                                                        // c = c & ~0xE0; // Clear self-test bits [7:5]
//    c = c & ~0x03; // Clear Fchoice bits [1:0]
//    c = c & ~0x18; // Clear GFS bits [4:3]
//    c = c | Gscale << 3; // Set full scale range for the gyro
//                         // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
//    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c); // Write new GYRO_CONFIG value to register
//
//                                                // Set accelerometer full-scale range configuration
//    c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
//                                                 // c = c & ~0xE0; // Clear self-test bits [7:5]
//    c = c & ~0x18;  // Clear AFS bits [4:3]
//    c = c | Ascale << 3; // Set full scale range for the accelerometer
//    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value
//
//                                                 // Set accelerometer sample rate configuration
//                                                 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
//                                                 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
//    c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
//    c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
//    c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
//    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value
//
//                                                  // Configure Interrupts and Bypass Enable
//                                                  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
//                                                  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
//                                                  // can join the I2C bus and all can be controlled by the Arduino as master
//    writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
//    writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
//    delay(100);
//}
//
//
//void initAK8963(float* destination)
//{
//    uint8_t rawData[3];
//
//    // Power down magnetometer to reset
//    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00);
//    delay(10);
//
//    // Extract the factory calibration for each magnetometer axis
//    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
//    delay(10);
//    readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
//    destination[0] = (float)(rawData[0] - 128) / 256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
//    destination[1] = (float)(rawData[1] - 128) / 256. + 1.;
//    destination[2] = (float)(rawData[2] - 128) / 256. + 1.;
//    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
//    delay(10);
//
//    // Configure the magnetometer for continuous read and highest resolution
//    // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
//    // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
//    writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
//    delay(10);
//}
//
//
//// Wire.h read and write protocols
//void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
//{
//    Wire.beginTransmission(address);  // Initialize the Tx buffer
//    Wire.write(subAddress);           // Put slave register address in Tx buffer
//    Wire.write(data);                 // Put data in Tx buffer
//    Wire.endTransmission();           // Send the Tx buffer
//}
//
//
//uint8_t readByte(uint8_t address, uint8_t subAddress)
//{
//    uint8_t data; // `data` will store the register data
//    Wire.beginTransmission(address);         // Initialize the Tx buffer
//    Wire.write(subAddress);                  // Put slave register address in Tx buffer
//    Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
//    Wire.requestFrom(address, (uint8_t)1);  // Read one byte from slave register address
//    data = Wire.read();                      // Fill Rx buffer with result
//    return data;                             // Return data read from slave register
//}
//
//
//void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
//{
//    Wire.beginTransmission(address);   // Initialize the Tx buffer
//    Wire.write(subAddress);            // Put slave register address in Tx buffer
//    Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
//
//    uint8_t i = 0;
//
//    Wire.requestFrom(address, count);  // Read bytes from slave register address
//
//    while (Wire.available())
//    {
//        dest[i++] = Wire.read();       // Put read results in the Rx buffer
//    }
//}
//

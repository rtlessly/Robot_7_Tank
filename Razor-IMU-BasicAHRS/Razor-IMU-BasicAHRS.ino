/*------------------------------------------------------------------------------
Sparkfun Razor IMU M0 AHRS Firmware

By:         R. T. Lessly
Date:       2018-05-25
License:    TBD

This is the source code for firmware to load on the SAMD21 microcontroller of the
Sparkfun Razor IMU M0 for it to act as a 3-axis Attitude Heading Reference System
(AHRS). This fimware configures the Razor to act as a slave device to another
computer via either a Serial port or an I2C port. The firmware is designed to respond
to a specific set of commands to return 3-axis scaled accelerometer, gyroscope,
magnetometer, Euler angels (yaw, pitch, roll), and velocity information.

All commands begin with "Q" and is followed by a letter indicating the requested
data. The letters are (must be upper case):
    - A : Return 3-axis acceleration vector in g's
    - G : Return 3-axis gyroscope rate vector in degrees/sec
    - M : Return 3-axis magnetometer vector in milliGuass
    - V : Return 3-axis integrated velocity vector in meters/sec
    - E : Return Euler angles as yaw, pitch, roll in degrees
    - I : Returns the Firmware ID (0x50)

For example, to request Euler angles, send the command "QE" via either I2C or
the serial port. NOTE: It is not necessary to null-terminate the command string
but it won't hurt anything if you do.

The coordinate axes of all vector responses align with the accelerometer and
gyroscope axes of the MPU-9250 (as documented on Razor circuit board). The
magnetometer axes are translated to align with the accelerometer and gyroscope so
that all vectors use the same coordinate axes. Note that this coordinate system 
is fixed to the Razor's reference frame.

The Euler angles make use of the Earth's magnetic field and gravity to determine 
an Earth-fixed frame of reference which can be used to compute an the absolute
orientation of the Razor relative to the Earth. Yaw is the the angle between the
Razor's x-axis and Earth's magnetic north, with 0/360 degrees pointing North, 90
degrees pointing east, etc. Pitch is the angle between the Razor's x-axis and the
tangential plane of the Earth surface directly below the vehicle (i.e., the horizon
plane). Positive pitch is when the Razor's x-axis is pointed up. Roll is the angle
between the Razor's z-axis and the gravity vector.

If using the serial port, the command is transmitted to the Razor as a string 
terminated with a CR/LF. The Razor sends the results back in ASCII format prefixed
with the requested command code (A, G, M, V, E, ID), followed by a space and then
the data with each number separated by a space. All numbers are formatted as
floating point with 2 decimal digits. The entire response is terminated with a
CR/LF. All vectors are sent back in X, Y, Z order, and the Euler angles are sent
back in yaw, pitch, roll order.

If using the I2C port, send the command string to address 0x40 followed by a data 
read request to the same address. The results are sent back as 4-byte binary floating 
point numbers in the native floating point byte-order of the SAMD21. All vectors
are sent back in X, Y, Z order, and the Euler angles are send back in yaw, pitch, 
roll order.

The firmware makes use of the open source Madgwick and Mahony filter algorithms 
to compute an absolute orientation quaternion that is then converted to Euler angles.

Since the primary I2C port of the SAMD21 is dedicated to communicating with the
on-board MPU-9250 IMU, and since SAMD21 I2C ports cannot be configured as both a
master and slave at the same time, this firmware configures an alternate I2C port
on the Razor's pin A3 (SDA) and A4 (SCL) using the SAMD21 SERCOM0 port to act as
the slave I2C port (see explanation below).

 Note: The Razor runs on the 3.3v and is not 5v tolerant. To connect it to a 5v
 system such as an Arduino Uno, a bidirectional signal level converter should be
 used to connect the I2C and/or the hardware serial port of the two devices.
 */
#include <Arduino.h>
#include <wiring_private.h> // pinPeripheral() function
#include <Wire.h>
#include <RTL_Stdlib.h>
#include <EventFramework.h>
#include "MPU9250.h"

#define RAZOR_IMU_ADDRESS   ((byte)0x40)
#define RAZOR_IMU__ID       ((byte)0x50)

#undef ConsoleStream
#define ConsoleStream SerialUSB

#ifndef DataStream
#define DataStream SerialUSB
#endif

#ifndef TelemetryStream
#define TelemetryStream SerialUSB
#endif


// Forward function declarations
void BlinkLEDCount(uint16_t count, uint16_t onTime=100, uint16_t offTime=100);
void IndicateFailure();
void ReceiveSerial(Stream& dataStream);
void SendTelemetry(Stream& dataStream);


// Pin definitions
int intPin =  4;            // Interrupt pin, 2 and 3 are the Arduino's ext int pins, SAMD uses pin 4
int HW_LED_PIN = 13;        // Set up pin 13 led for toggling

// The all-important MPU-9250 IMU
MPU9250 imu;

// Slave I2C connection on SERCOM0 (A3=SDA, A4=SCL)
TwoWire  slaveI2C = TwoWire(&sercom0, A3, A4);

// IMU sensor data values
ScaledData accel;           // Scaled accelerometer vector (m/s^2)
ScaledData gyro;            // Scaled gyro rates (degrees/sec)
ScaledData mag;             // Scaled magnetometer vector (milliGaus)
ScaledData velocity;        // Integrated velocity vector (m/s)
ScaledData position;        // Integrated position vector (m)

// Housekeeping variables
uint32_t lastUpdate = 0;    // Time of previous iteration - used to calculate integration interval
uint32_t countT = 0;        // Number of integration intervals in a engineering reporting interval
float    deltaT = 0.0f;     // integration time interval for filter (microseconds)
float    sumT = 0.0f;       // Total integration time in a engineering reporting interval (microseconds)


void setup()
{
    float   selfTestResults[6];                 // Results of gyro and accelerometer self test
                                                
    // Start ConsoleStream first thing so we can see diagnostics
    ConsoleStream.begin(115200);
    while (!ConsoleStream && millis() < 1000);  // Wait for SerialUSB to initialize (max 1 second since startup)

    // Start I2C interface, aka Two-Wire Interface (TWI)
    Wire.begin(); 
    //TWBR = 12;  // Sets 400 kbit/sec I2C speed

    /*--------------------------------------------------------------------------
    Initiate I2C slave port on SERCOM0 
    The Sparkfun Razor IMU M0 uses a Atmel SAMD21 microcontroller, which has 
    specialized on-chip hardware for serial communications. Because of this, the
    SAMD21 Wire library is NOT fully compatible with the standard Arduino WIRE
    library. Specifically, the I2C ports on this chip cannot be configured as both
    a master and a slave at the same time (as can be done on an Uno or similar
    board). In addition, The default Wire port on the Razor must be used in master
    mode to communicate with the MPU-9250 and the AK8963, which means it cannot 
    be used for the Razor to communicate as a slave with an external board. This
    means we need another I2C port that can be configured as an I2C slave.

    Fortuanately, the design of the SAMD21 includes 6 Serial Communication ports
    (called SERCOMs) that can each be configured to be either a standard serial
    (RX and TX), I2C, or SPI port. Unfortunately, due to the limited number of pins
    avaiable on the Razor board, the only pins available that can be used for a
    second I2C port are A3 and A4, which are mapped to the SERCOM0 serial port on
    the SAMD21. Therefore, the A3 and A4 pins must be reprogrammed to be used for
    SERCOM0 I2C SDA and SCL functionality.

    Note that using SERCOM0 in this way makes the hardware serial port on pins
    D0 and D1 (aka Serial on the Razor), unavailable since it also uses SERCOM0.
    However, the Serial USB port (aka SerialUSB) will still work just fine. Also,
    just as we are reporgramming SERCOM0 as a second I2C port, we could reprogram
    another SERCOM as an alternate serial port, if needed.
    --------------------------------------------------------------------------*/
    slaveI2C.begin(RAZOR_IMU_ADDRESS);

    // Must reassign pins A3 & A4 to SERCOM functionality
    // BUT it has to be done after the begin() call!
    pinPeripheral(A3, PIO_SERCOM_ALT);      // A3=SDA
    pinPeripheral(A4, PIO_SERCOM_ALT);      // A4=SCL

    // Configure Slave interrupt handlers
    slaveI2C.onReceive(ReceiveI2C);         // interrupt handler for incoming messages
    slaveI2C.onRequest(RequestI2C);         // interrupt handler for data requests

    // Configure interrupt pin, its set as active high, push-pull
    pinMode(intPin, INPUT);
    digitalWrite(intPin, LOW);

    // Configure LED pin
    pinMode(HW_LED_PIN, OUTPUT);
    digitalWrite(HW_LED_PIN, LOW);

    if (!imu.TestConnection(TelemetryStream))
    {
        ConsoleStream << "Unable to connect to MPU-9250, Aborting..." << endl;

        // Loop forever if communication doesn't happen
        while (true) IndicateFailure();
    }

    imu.SelfTest(selfTestResults); // Start by performing self test and reporting values
    imu.Calibrate(TelemetryStream);
    imu.Initialize(TelemetryStream);

    TelemetryStream << endl;
    TelemetryStream << "MPU-9250 self-test results:" << endl;
    TelemetryStream << "    Accelerometer X-axis trim within " << _FLOAT(selfTestResults[0],1) << "% of factory value" << endl;
    TelemetryStream << "    Accelerometer Y-axis trim within " << _FLOAT(selfTestResults[1],1) << "% of factory value" << endl;
    TelemetryStream << "    Accelerometer Z-axis trim within " << _FLOAT(selfTestResults[2],1) << "% of factory value" << endl;
    TelemetryStream << "    Gyroscope X-axis trim within "     << _FLOAT(selfTestResults[3],1) << "% of factory value" << endl;
    TelemetryStream << "    Gyroscope Y-axis trim within "     << _FLOAT(selfTestResults[4],1) << "% of factory value" << endl;
    TelemetryStream << "    Gyroscope Z-axis trim within "     << _FLOAT(selfTestResults[5],1) << "% of factory value" << endl;

    float magBias[3];
    float magSens[3];

    imu.GetMagCalibration(magBias, magSens);

    TelemetryStream << endl;
    TelemetryStream << "Magnetometer calibration values: " << endl;
    TelemetryStream << "    X-Axis sensitivity adjustment value " << _FLOAT(magSens[0], 2) << endl;
    TelemetryStream << "    Y-Axis sensitivity adjustment value " << _FLOAT(magSens[1], 2) << endl;
    TelemetryStream << "    Z-Axis sensitivity adjustment value " << _FLOAT(magSens[2], 2) << endl;

    TelemetryStream << endl;
    TelemetryStream << "MPU-9250 is online..." << endl;
}


void loop()
{
    auto prevAccel = accel;

    // Ask the IMU to update its raw state vectors
    imu.Update();

    // Get the current scaled and calibrated state vectors
    imu.GetAccel(accel);
    imu.GetGyro(gyro);
    imu.GetMag(mag);

    auto now = micros();
    
    // set integration time by time elapsed since last filter update
    deltaT = (now - lastUpdate) / 1000000.0f;
    lastUpdate = now;

    // sum for averaging filter update rate
    sumT += deltaT; 
    countT++;

    // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
    // the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
    // We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
    // For the MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
    // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
    // This is ok by aircraft orientation standards!
    // Pass gyro rate as rad/s
    //MadgwickQuaternionUpdate(accel.X, accel.Y, accel.Z, gyro.X*PI/180.0f, gyro.Y*PI/180.0f, gyro.Z*PI/180.0f, mag.Y, mag.X, -mag.Z, deltaT);
    MahonyQuaternionUpdate(accel.X, accel.Y, accel.Z, gyro.X*PI/180.0f, gyro.Y*PI/180.0f, gyro.Z*PI/180.0f, mag.Y, mag.X, -mag.Z, deltaT);
    UpdateEulerAngles();

    // Update deltaT to account for time in quaternion update
    deltaT += (micros() - lastUpdate) / 1000000.0f;

    const float one_g = 9.80665; // Converts g's to m/s^2

    // Integrate velocity
    velocity.X += ((accel.X + prevAccel.X) / 2) * one_g * deltaT;
    velocity.Y += ((accel.Y + prevAccel.Y) / 2) * one_g * deltaT;
    velocity.Z += ((accel.Z + prevAccel.Z) / 2) * one_g * deltaT;

    // Check for and process incoming serial requests
    if (DataStream.available())
    {
        ReceiveSerial(DataStream);
        RequestSerial(DataStream);

        // Toggle LED to indicate communication occurred
        digitalWrite(HW_LED_PIN, !digitalRead(HW_LED_PIN));
    }
        
    // Send engineering telemetry to serial port at 5Hz
    //SendTelemetry(TelemetryStream);
}


// Buffer to hold command messages
uint8_t commandBuffer[33];


//****************************************************************************
// IRQ handler for recieving a message on Sercom0
//****************************************************************************
void SERCOM0_Handler()
{
    slaveI2C.onService();
}


//****************************************************************************
// Interrupt handler for recieving a message on slave I2C connection
//****************************************************************************
static void ReceiveI2C(int messageLength)
{
    auto maxLength = min(sizeof(commandBuffer) - 1, messageLength);
    auto bytesRead = 0;

    while (bytesRead < maxLength) commandBuffer[bytesRead++] = slaveI2C.read();

    commandBuffer[bytesRead] = 0;
}


//****************************************************************************
// Interrupt handler for responding to a request on slave I2C connection
//****************************************************************************
static void RequestI2C()
{
    if (commandBuffer[0] == 'Q')
    {
        switch (commandBuffer[1])
        {
            case 'E': 
                slaveI2C.write((uint8_t*)&eulerAngles, sizeof(eulerAngles));
                break;

            case 'A':
                slaveI2C.write((uint8_t*)&accel, sizeof(accel));
                break;

            case 'G':
                slaveI2C.write((uint8_t*)&gyro, sizeof(gyro));
                break;

            case 'M':
                slaveI2C.write((uint8_t*)&mag, sizeof(mag));
                break;

            case 'V':
                slaveI2C.write((uint8_t*)&velocity, sizeof(velocity));
                break;

            case 'I':
                slaveI2C.write(RAZOR_IMU__ID);
                break;

            default:
                slaveI2C.write(0);
                break;
        }
    }
    else
    {
        slaveI2C.write(0);
    }

    commandBuffer[0] = 0;
}


//****************************************************************************
// Handler for recieving a message on serial connection
//****************************************************************************
static void ReceiveSerial(Stream& dataStream)
{
    if (!dataStream.available()) return;

    auto bytesRead = dataStream.readBytesUntil('\n', commandBuffer, sizeof(commandBuffer)-1);

    commandBuffer[bytesRead] = 0;
}


//****************************************************************************
// Handler for responding to a request on serial connection
//****************************************************************************
void RequestSerial(Stream& dataStream)
{
    if (commandBuffer[0] == 0) return;

    if (commandBuffer[0] == 'Q')
    {
        switch (commandBuffer[1])
        {
            case 'E':             // Send Euler angles
                DataStream << "E " << _FLOAT(yaw, 2) << " " << _FLOAT(pitch, 2) << " " << _FLOAT(roll, 2) << endl;
                break;

            case 'A':             // Send acceleration vector
                DataStream << "A " << _FLOAT(accel.X, 2) << " " << _FLOAT(accel.Y, 2) << " " << _FLOAT(accel.Z, 2) << endl;
                break;

            case 'G':             // Send gyro rates vector
                DataStream << "G " << _FLOAT(gyro.X, 2) << " " << _FLOAT(gyro.Y, 2) << " " << _FLOAT(gyro.Z, 2) << endl;
                break;

            case 'M':             // Send magnetometer vector
                DataStream << "M " << _FLOAT(mag.X, 2) << " " << _FLOAT(mag.Y, 2) << " " << _FLOAT(mag.Z, 2) << endl;
                break;

            case 'V':             // Send velocity vector
                DataStream << "V " << _FLOAT(velocity.X, 2) << " " << _FLOAT(velocity.Y, 2) << " " << _FLOAT(velocity.Z, 2) << endl;
                break;

            case 'I':
                DataStream << "I " << _HEX(RAZOR_IMU__ID) << endl;
                break;

            default:
                DataStream << endl;
                break;
        }
    }
    else
    {
        DataStream << endl;
    }

    commandBuffer[0] = 0;
}


void SendTelemetry(Stream& telemtryStream)
{
    static uint32_t lastTime = 0;

    auto updateRate = 0.0F; 
    auto now = millis();
    auto dt = now - lastTime;
    
    if (dt < 200) return;

    auto tempCount = imu.ReadTempData();            // Read the temperature sensor
    auto temperature = tempCount / 333.87 + 21.0;   // Convert to Celcius

    UpdateEulerAngles();
    
    // With these settings the filter is updating at a ~145 Hz rate using the Madgwick scheme and
    // >200 Hz using the Mahony scheme even though the display refreshes at only 5 Hz.
    // The filter update rate is determined mostly by the mathematical steps in the respective algorithms,
    // the processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer ODR:
    // an ODR of 10 Hz for the magnetometer produce the above rates, maximum magnetometer ODR of 100 Hz produces
    // filter update rates of 36 - 145 and ~38 Hz for the Madgwick and Mahony schemes, respectively.
    // This is presumably because the magnetometer read takes longer than the gyro or accelerometer reads.
    // This filter update rate should be fast enough to maintain accurate platform orientation for
    // stabilization control of a fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
    // produced by the on-board Digital Motion Processor of Invensense's MPU6050 6 DoF and MPU9150 9DoF sensors.
    // The 3.3 V 8 MHz Pro Mini is doing pretty well!
    if (sumT > 0)
    {
        updateRate = countT / sumT;
        countT = 0;
        sumT = 0;
    }
    
    telemtryStream << endl;
    telemtryStream << "@UR " << _FLOAT(updateRate, 2) << " Hz" << endl;
    telemtryStream << "@EA " << _FLOAT(yaw, 2) << " " << _FLOAT(pitch, 2) << " " << _FLOAT(roll, 2) << " deg"    << endl;
    telemtryStream << "@AB " << _FLOAT(accel.X,3) << " " << _FLOAT(accel.Y,3) << " " << _FLOAT(accel.Z,3)          << " G"      << endl; 
    telemtryStream << "@GB " << _FLOAT(gyro.X,3) << " " << _FLOAT(gyro.Y,3) << " " << _FLOAT(gyro.Z,3)          << " deg/s"  << endl; 
    telemtryStream << "@MB " << _FLOAT(mag.X,0) << " " << _FLOAT(mag.Y,0) << " " << _FLOAT(mag.Z,0)          << " mGauss" << endl; 
    telemtryStream << "@TP " << _FLOAT(temperature, 1)                                              << " C"      << endl; 

    lastTime = now;
}


void BlinkLEDCount(uint16_t count, uint16_t onTime, uint16_t offTime)
{
    for (int i = 0; i < count; i++)
    {
      digitalWrite(HW_LED_PIN, HIGH);
      delay(onTime);
      digitalWrite(HW_LED_PIN, LOW);
      delay(offTime);
    }
}


void IndicateFailure()
{
    // This flashes "S-O-S" in morse code
    BlinkLEDCount(3, 100, 150);
    delay(100);
    BlinkLEDCount(3, 400, 150);
    delay(100);
    BlinkLEDCount(3, 100, 150);
    delay(500);
}

/*------------------------------------------------------------------------------
RTL_IMU implementation.

By:         R. T. Lessly
Date:       2018-05-25
License:    TBD

This is the firmware for the RTL_IMU to load on the SAMD21 microcontroller of the
Sparkfun Razor IMU M0 for it to act as a 3-axis IMU/AHRS (Inertial Measurement Unit /
Attitude Heading Reference System). This fimware configures the Razor to act as 
a slave device to another computer via either a Serial port or an I2C port. The 
firmware is designed to respond to a specific set of commands to return 3-axis 
scaled accelerometer, gyroscope, magnetometer, Euler angles (yaw, pitch, roll), 
compass heading, and velocity information.

See RTL_IMU.h for information on the I2C/Serial command codes and sequences recognized
by this IMU implementation.

The coordinate axes of all vector responses align with the accelerometer and
gyroscope axes of the MPU-9250 (as documented on Razor circuit board). The
magnetometer axes are translated to align with the accelerometer and gyroscope so
that all vectors use the same coordinate axes. Note that this coordinate system 
is fixed to the Razor's reference frame.

The Euler angles make use of the Earth's magnetic field and gravity to determine 
an Earth-fixed frame of reference which can be used to compute an the absolute
orientation of the Razor relative to the Earth. The angles are defined as follows:

    * Yaw is the angle between the Razor x-axis and Earth magnetic North (or true
      North if corrected for local declination), with 0/360 degrees pointing North,
      90 degrees pointing east, etc. Looking down on the sensor positive yaw is 
      counterclockwise.

    * Pitch is angle between the Razor x-axis and Earth ground plane, toward the 
      Earth is positive, up toward the sky is negative.

    * Roll is angle between sensor y-axis and Earth ground plane, y-axis up is 
      positive roll.

If using the serial port, the command is transmitted to the Razor as a string 
terminated with a CR/LF. The Razor sends the results back in ASCII format prefixed
with the requested command code (A, a, G, g, m V, E, I), followed by a space and
then the data with each number separated by a space. All numbers are formatted as
floating point with 2 decimal digits. The entire response is terminated with a
CR/LF. All vectors are sent back in X, Y, Z order, and the Euler angles are sent
back in yaw, pitch, roll order.

If using the I2C port, send the command string to address 0x40 followed by a data 
read request to the same address. The results are sent back as 4-byte binary floating 
point numbers in the native floating point byte-order of the SAMD21. All vectors
are sent back in X, Y, Z order, and the Euler angles are send back in yaw, pitch, 
roll order.

The firmware makes use of the open source Madgwick and Mahony filter algorithms 
to compute an absolute orientation quaternion that can then be converted to Euler
angles.

Since the primary I2C port of the SAMD21 is dedicated to communicating with the
on-board MPU-9250 IMU, and since SAMD21 I2C ports cannot be configured as both a
master and slave at the same time, this firmware configures an alternate I2C port
on the Razor's pin A3 (SDA) and A4 (SCL) using the SAMD21 SERCOM0 port to act as
the slave I2C port (see explanation below).

 Note: The Razor runs on the 3.3v and is not 5v tolerant. To connect it to a 5v
 system such as an Arduino Uno, a bidirectional signal level converter should be
 used to connect the I2C and/or the hardware serial port of the two devices.
 -----------------------------------------------------------------------------*/
#include <Arduino.h>
#include <wiring_private.h> // To get pinPeripheral() function
#include <Wire.h>
#include <RTL_Stdlib.h>
#include <EventFramework.h>
#include "RTL_IMU.h"
#include "MPU9250.h"

#define RAZOR_IMU_ADDRESS   ((byte)0x40)
#define RAZOR_IMU_ID       ((byte)0x50)


#undef ConsoleStream
#define ConsoleStream SerialUSB

#ifndef DataStream
#define DataStream SerialUSB
#endif


 // Forward function declarations
void ReceiveSerial(Stream& dataStream);
void RequestSerialID(Stream & dataStream);
void RequestSerialIsReady(Stream & dataStream);
void ProcessContinuousModeCommand(Stream& dataStream, const char command, const char param1, const char param2);
void RequestSerialAccel(Stream& dataStream);
void RequestSerialDynAccel(Stream & dataStream);
void RequestSerialGyro(Stream & dataStream);
void RequestSerialGimbal(Stream & dataStream);
void RequestSerialMag(Stream & dataStream);
void RequestSerialVelocity(Stream & dataStream);
void RequestSerialEuler(Stream & dataStream);
void BlinkLEDCount(uint16_t count, uint16_t onTime = 100, uint16_t offTime = 100);
void IndicateFailure();



// Pin definitions
int intPin =  4;            // Interrupt pin, 2 and 3 are the Arduino's ext int pins, SAMD uses pin 4
int HW_LED_PIN = 13;        // Set up pin 13 led for toggling

// Slave I2C connection on SERCOM0 (A3=SDA, A4=SCL)
TwoWire  slaveI2C = TwoWire(&sercom0, A3, A4);

// The all-important MPU-9250 IMU
MPU9250 imu;

// IMU sensor data values
Vector3F accel;              // Scaled accelerometer vector (g)
Vector3F gyro;               // Scaled gyro rates (radians/sec)
Vector3F mag;                // Scaled magnetometer vector (milliGaus)
Vector3F dynAccel;           // Dynamic acceleration (gravity component removed)
Vector3F velocity;           // Integrated velocity vector (m/s)
Vector3F gimbal;             // Integrated gimbal angles (degrees)

// Housekeeping variables
uint32_t lastUpdate = 0;    // Time of previous iteration - used to calculate integration interval
bool ready = false;         // true=the AHRS is ready to work, false=still initializing

// Indicates which values should be output in continuous mode
bool modeContinuous = false;
bool accelContinuous = false;
bool gyroContinuous = false;
bool magContinuous = false;
bool eulerContinuous = false;
bool dynAccelContinuous = false;
bool gimbalContinuous = false;
bool headingContinuous = false;
bool velocityContinuous = false;


//******************************************************************************
// IRQ handler for recieving a message on Sercom0
//******************************************************************************
void SERCOM0_Handler()
{
    slaveI2C.onService();
}


void setup()
{
    // Start ConsoleStream first thing so we can see diagnostics
    ConsoleStream.begin(115200);

    // Wait for SerialUSB to initialize (max 1 second since startup)
    while (!ConsoleStream && millis() < 1000);

    ConsoleStream << "RTL_IMU_Razor: In setup()..." << endl;

    // Start I2C interface, aka Two-Wire Interface (TWI)
    ConsoleStream << "RTL_IMU_Razor: begin I2C Master..." << endl;
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
    mode to communicate with the MPU-9250, which means it cannot be used for the
    Razor to communicate as a slave with an external board. This means we need
    another I2C port that can be configured as an I2C slave.

    Fortuanately, the design of the SAMD21 includes 6 serial communication ports
    (called SERCOMs, designated SERCOM0 to SERCOM5) that can each be configured
    to be either a standard serial (RX and TX), I2C, or SPI port. Unfortunately,
    due to the limited number of pins broken out on the Razor board, the only pins
    available that can be used for a second I2C port are A3 and A4, which are
    mapped to the I2C function of SERCOM0. Therefore, the A3 and A4 pins must be
    reprogrammed to be used for SERCOM0 I2C SDA and SCL functionality.

    Note that using SERCOM0 in this way makes the hardware serial port on pins
    D0 and D1 (aka Serial on the Razor), unavailable since it also uses SERCOM0.
    However, the Serial USB port (aka SerialUSB) will still work just fine. Also,
    just as we are reporgramming SERCOM0 as a second I2C port, we could reprogram
    another SERCOM as an alternate serial port, if needed.
    --------------------------------------------------------------------------*/
    ConsoleStream << "RTL_IMU_Razor: begin I2C Slave..." << endl;
    slaveI2C.begin(RAZOR_IMU_ADDRESS);

    // Must reassign pins A3 & A4 to SERCOM functionality
    // BUT it has to be done after the begin() call!
    pinPeripheral(A3, PIO_SERCOM_ALT);      // A3=SDA
    pinPeripheral(A4, PIO_SERCOM_ALT);      // A4=SCL

    // Configure Slave interrupt handlers
    slaveI2C.onReceive(ReceiveI2C);         // interrupt handler for incoming messages
    slaveI2C.onRequest(RequestI2C);         // interrupt handler for data requests

    // Configure MPU-9250 interrupt pin, its set as active high, push-pull
    ConsoleStream << "RTL_IMU_Razor: Configure pins..." << endl;
    pinMode(intPin, INPUT);
    digitalWrite(intPin, LOW);

    // Configure LED pin
    pinMode(HW_LED_PIN, OUTPUT);
    digitalWrite(HW_LED_PIN, LOW);

    // Verify connectivity with MPU-9250
    ConsoleStream << "RTL_IMU_Razor: Test MPU-9250 connectivity..." << endl;

    if (!imu.TestConnection(ConsoleStream))
    {
        ConsoleStream << "Unable to connect to MPU-9250, Aborting..." << endl;

        // Loop forever if communication doesn't happen
        while (true) IndicateFailure();
    }

    // Results of gyro and accelerometer self test
    float selfTestResults[6];

    // Perform self-test, self-calibration, and initialization of MPU-9250
    ConsoleStream << "RTL_IMU_Razor: Performing self test..." << endl;
    imu.SelfTest(selfTestResults);
    ConsoleStream << "RTL_IMU_Razor: Performing calibration..." << endl;
    imu.Calibrate(ConsoleStream);
    ConsoleStream << "RTL_IMU_Razor: Starting IMU..." << endl;
    imu.Begin(ConsoleStream);

    ConsoleStream << endl;
    ConsoleStream << "MPU-9250 self-test results:" << endl;
    ConsoleStream << "    Accelerometer X-axis trim within " << _FLOAT(selfTestResults[0],1) << "% of factory value" << endl;
    ConsoleStream << "    Accelerometer Y-axis trim within " << _FLOAT(selfTestResults[1],1) << "% of factory value" << endl;
    ConsoleStream << "    Accelerometer Z-axis trim within " << _FLOAT(selfTestResults[2],1) << "% of factory value" << endl;
    ConsoleStream << "    Gyroscope X-axis trim within "     << _FLOAT(selfTestResults[3],1) << "% of factory value" << endl;
    ConsoleStream << "    Gyroscope Y-axis trim within "     << _FLOAT(selfTestResults[4],1) << "% of factory value" << endl;
    ConsoleStream << "    Gyroscope Z-axis trim within "     << _FLOAT(selfTestResults[5],1) << "% of factory value" << endl;

    float magBias[3];
    float magSens[3];

    imu.GetMagCalibration(magBias, magSens);

    ConsoleStream << endl;
    ConsoleStream << "Magnetometer calibration values: " << endl;
    ConsoleStream << "    X-Axis sensitivity adjustment value " << _FLOAT(magSens[0], 2) << endl;
    ConsoleStream << "    Y-Axis sensitivity adjustment value " << _FLOAT(magSens[1], 2) << endl;
    ConsoleStream << "    Z-Axis sensitivity adjustment value " << _FLOAT(magSens[2], 2) << endl;

    ConsoleStream << endl;
    ConsoleStream << "MPU-9250 is online..." << endl;
    ready = true;
}


void loop()
{
    // Remember gyro state vector from previous iteration
    auto prevGyro = gyro;

    // Ask the IMU to update its state vectors
    imu.Update();

    // Get new scaled state vectors
    accel = imu.GetAccel();
    gyro = imu.GetGyro();
    mag = imu.GetMag();

    // set integration time by time elapsed since last update
    auto now = micros();
    auto deltaT = (now - lastUpdate) / 1000000.0f;

    lastUpdate = now;

    //MadgwickQuaternionUpdate(accel.x, accel.y, accel.z, gyro.x*PI/180.0f, gyro.y*PI/180.0f, gyro.z*PI/180.0f, mag.y, mag.x, -mag.z, deltaT);
    //MahonyQuaternionUpdate(accel.x, accel.y, accel.z, gyro.x*PI/180.0f, gyro.y*PI/180.0f, gyro.z*PI/180.0f, mag.x, mag.y, mag.z, deltaT);

    // NOTE: The Mahony filter returns an updated gyro rate vector to correct for gyro drift
    gyro = MahonyQuaternionUpdate(accel, gyro, mag, deltaT);
    UpdateEulerAngles();

    // Integrate velocity
    auto a = imu.GetDynamicAccel();
    auto f = 0.5f * ONE_G * deltaT;

    velocity.x += (a.x + dynAccel.x) * f;
    velocity.y += (a.y + dynAccel.y) * f;
    velocity.z += (a.z + dynAccel.z) * f;
    dynAccel = a;

    // Integrate gyro rates to update gimbal angles in degrees
    f = 0.5 * (180.0 / PI) * deltaT;
    gimbal.x += (gyro.x + prevGyro.x) * f;
    gimbal.y += (gyro.y + prevGyro.y) * f;
    gimbal.z += (gyro.z + prevGyro.z) * f;

    // Check for and process incoming serial requests
    if (DataStream.available())
    {
        ReceiveSerial(DataStream);
        RequestSerial(DataStream);
    }

    if (modeContinuous)
    {
        if (accelContinuous) RequestSerialAccel(DataStream);

        if (gyroContinuous) RequestSerialGyro(DataStream);
        
        if (magContinuous) RequestSerialMag(DataStream);

        if (eulerContinuous) RequestSerialEuler(DataStream);

        if (dynAccelContinuous) RequestSerialDynAccel(DataStream);

        if (velocityContinuous) RequestSerialVelocity(DataStream);

        if (gimbalContinuous) RequestSerialGimbal(DataStream);

        if (headingContinuous) RequestSerialHeading(DataStream);

        // Toggle LED to indicate continuous mode active
        digitalWrite(HW_LED_PIN, !digitalRead(HW_LED_PIN));
    }
}


// Buffer to hold command messages
uint8_t commandBuffer[33];


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
    if (commandBuffer[0] == CMD_PREFIX)
    {
        switch (commandBuffer[1])
        {
            case CMD_ID:
                slaveI2C.write(RAZOR_IMU_ID);
                break;

            case CMD_IS_READY:
                slaveI2C.write(ready ? 1 : 0);
                break;

            case CMD_ACCEL:
                slaveI2C.write((uint8_t*)&accel, sizeof(accel));
                break;

            case CMD_DYN_ACCEL:
                slaveI2C.write((uint8_t*)&dynAccel, sizeof(dynAccel));
                break;

            case CMD_GYRO:
                slaveI2C.write((uint8_t*)&gyro, sizeof(gyro));
                break;

            case CMD_GIMBAL:
                slaveI2C.write((uint8_t*)&gimbal, sizeof(gimbal));
                break;

            case CMD_MAG:
                slaveI2C.write((uint8_t*)&mag, sizeof(mag));
                break;

            case CMD_VELOCITY:
                slaveI2C.write((uint8_t*)&velocity, sizeof(velocity));
                break;

            case CMD_EULER:
                slaveI2C.write((uint8_t*)&eulerAngles, sizeof(eulerAngles));
                break;

            case CMD_ZERO:
                velocity.x = velocity.y = velocity.z = 0.0;
                break;

            default:
                slaveI2C.write((uint8_t)0xFE);   // 0xFE=Unrecognized command code
                break;
        }
    }
    else
    {
        slaveI2C.write((uint8_t)0xFF);           // 0xFF=Unrecognized command string
    }

    // Clears command buffer
    commandBuffer[0] = 0;
}


//****************************************************************************
// Handler for recieving a message on serial connection
//****************************************************************************
void ReceiveSerial(Stream& dataStream)
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

    if (commandBuffer[0] == CMD_PREFIX)
    {
        auto command = commandBuffer[1];

        switch (command)
        {
            case CMD_ID:
                RequestSerialID(dataStream);
                break;

            case CMD_IS_READY:
                RequestSerialIsReady(dataStream);
                break;

                // Send acceleration vector
            case CMD_ACCEL:
                RequestSerialAccel(dataStream);
                break;

            // Send dynamic acceleration vector
            case CMD_DYN_ACCEL:
                RequestSerialDynAccel(dataStream);
                break;

            // Send gyro rates vector
            case CMD_GYRO:
                RequestSerialGyro(dataStream);
                break;

            // Send gimbal angles vector
            case CMD_GIMBAL:
                RequestSerialGimbal(dataStream);
                break;

            // Send magnetometer vector
            case CMD_MAG:
                RequestSerialMag(dataStream);
                break;

            // Send velocity vector
            case CMD_VELOCITY:
                RequestSerialVelocity(dataStream);
                break;

            // Send Euler angles
            case CMD_EULER:
                RequestSerialEuler(dataStream);
                break;

            case CMD_ZERO:
                velocity.x = velocity.y = velocity.z = 0.0;
                break;

            case CMD_HEADING:
                RequestSerialHeading(dataStream);
                break;

            case CMD_CONTINUOS:
            {
                auto param1 = (char)commandBuffer[2];
                auto param2 = (char)commandBuffer[3];

                ProcessContinuousModeCommand(dataStream, command, param1, param2);
                break;
            }

            default:
                dataStream << "0xFE" << endl;   // 0xFE=Unrecognized command code
                break;
        }
    }
    else
    {
        dataStream << "0xFF" << endl;           // 0xFF=Unrecognized command string
    }

    // Clears command buffer
    commandBuffer[0] = 0;
}


void ProcessContinuousModeCommand(Stream& dataStream, const char command, const char param1, const char param2)
{
    switch (param1)
    {
    case '1':
    case '0':
        modeContinuous = param1 == '1';
        dataStream << '!' << command << ' ' << modeContinuous << endl;
        break;
    case '9':
    {
        auto enableAll = param1 == '1';

        accelContinuous = enableAll;
        gyroContinuous = enableAll;
        magContinuous = enableAll;
        eulerContinuous = enableAll;
        dynAccelContinuous = enableAll;
        velocityContinuous = enableAll;
        gimbalContinuous = enableAll;
        dataStream << '!' << command << ' ' << enableAll << endl;
        break;
    }
    case CMD_ACCEL:
        accelContinuous = param2 == '1';
        dataStream << '!' << command << param1 << ' ' << accelContinuous << endl;
        break;
    case CMD_GYRO:
        gyroContinuous = param2 == '1';
        dataStream << '!' << command << param1 << ' ' << gyroContinuous << endl;
        break;
    case CMD_MAG:
        magContinuous = param2 == '1';
        dataStream << '!' << command << param1 << ' ' << magContinuous << endl;
        break;
    case CMD_EULER:
        eulerContinuous = param2 == '1';
        dataStream << '!' << command << param1 << ' ' << eulerContinuous << endl;
        break;
    case CMD_DYN_ACCEL:
        dynAccelContinuous = param2 == '1';
        dataStream << '!' << command << param1 << ' ' << dynAccelContinuous << endl;
        break;
    case CMD_VELOCITY:
        velocityContinuous = param2 == '1';
        dataStream << '!' << command << param1 << ' ' << velocityContinuous << endl;
        break;
    case CMD_GIMBAL:
        gimbalContinuous = param2 == '1';
        dataStream << '!' << command << param1 << ' ' << gimbalContinuous << endl;
        break;
    case CMD_HEADING:
        headingContinuous = param2 == '1';
        dataStream << '!' << command << param1 << ' ' << headingContinuous << endl;
        break;
    }
}


void RequestSerialID(Stream & dataStream)
{
    dataStream << '#' << CMD_ID << ' ' << _HEX(RAZOR_IMU_ID) << endl;
}


void RequestSerialIsReady(Stream & dataStream)
{
    dataStream << '#' << CMD_IS_READY << ' ' << (ready ? '1' : '0') << endl;
}


void RequestSerialAccel(Stream& dataStream)
{
    dataStream << '#' << CMD_ACCEL << ' ' << _FLOAT(accel.x, 2) << ' ' << _FLOAT(accel.y, 2) << ' ' << _FLOAT(accel.z, 2) << endl;
}


void RequestSerialGyro(Stream & dataStream)
{
    dataStream << '#' << CMD_GYRO << ' ' << _FLOAT(gyro.x, 2) << ' ' << _FLOAT(gyro.y, 2) << ' ' << _FLOAT(gyro.z, 2) << endl;
}


void RequestSerialMag(Stream & dataStream)
{
    //auto magRaw = imu.GetMagRaw();

    dataStream << '#' << CMD_MAG << ' ' << _FLOAT(mag.x, 2) << ' ' << _FLOAT(mag.y, 2) << ' ' << _FLOAT(mag.z, 2) << endl;
    //dataStream << '#' << CMD_MAG << ' ' << magRaw.x << ' ' << magRaw.y << ' ' << magRaw.z << endl;
}


void RequestSerialEuler(Stream & dataStream)
{
    dataStream << '#' << CMD_EULER << ' ' << _FLOAT(eulerAngles.Yaw, 2) << ' ' << _FLOAT(eulerAngles.Pitch, 2) << ' ' << _FLOAT(eulerAngles.Roll, 2) << endl;
}


void RequestSerialHeading(Stream & dataStream)
{
    auto heading = imu.GetCompassHeadingDegrees();

    dataStream << '#' << CMD_HEADING << ' ' << _FLOAT(heading, 2) << endl;
}


void RequestSerialVelocity(Stream & dataStream)
{
    dataStream << '#' << CMD_VELOCITY << ' ' << _FLOAT(velocity.x, 2) << ' ' << _FLOAT(velocity.y, 2) << ' ' << _FLOAT(velocity.z, 2) << endl;
}


void RequestSerialGimbal(Stream & dataStream)
{
    dataStream << '#' << CMD_GIMBAL << ' ' << _FLOAT(gimbal.x, 2) << ' ' << _FLOAT(gimbal.y, 2) << ' ' << _FLOAT(gimbal.z, 2) << endl;
}


void RequestSerialDynAccel(Stream & dataStream)
{
    dataStream << '#' << CMD_DYN_ACCEL << ' ' << _FLOAT(dynAccel.x, 2) << ' ' << _FLOAT(dynAccel.y, 2) << ' ' << _FLOAT(dynAccel.z, 2) << endl;
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

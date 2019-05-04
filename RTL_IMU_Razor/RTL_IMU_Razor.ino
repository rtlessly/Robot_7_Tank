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
------------------------------------------------------------------------------*/
#include<cstring>
#include <Streaming.h>
#include <Arduino.h>
#include <wiring_private.h> // To get pinPeripheral() function
#include <Wire.h>
#include <RTL_Stdlib.h>
#include <RTL_EventFramework.h>
#include "RTL_IMU.h"
#include "MPU9250.h"
#include "SafePrint.h"


bool DEBUG_I2C = false;


#define RAZOR_IMU_ADDRESS   ((byte)0x40)
#define RAZOR_IMU_ID       ((byte)0x50)


#undef ConsoleStream
#define ConsoleStream SerialUSB

#ifndef DataStream
#define DataStream SerialUSB
#endif


const uint32_t UpdateInterval = 8000;   // update the IMU state vectors every 8 ms (125 times a sec)


// Forward function declarations
void ReceiveSerial(Stream& dataStream);
void RequestSerialID(Stream& dataStream);
void RequestSerialIsReady(Stream& dataStream);
void ProcessContinuousModeCommand(Stream& dataStream, const char command, const char param1, const char param2);
void RequestSerialAccel(Stream& dataStream, bool modeRaw=false);
void RequestSerialGyro(Stream& dataStream, bool modeRaw = false);
void RequestSerialMag(Stream& dataStream, bool modeRaw = false);
void RequestSerialEuler(Stream& dataStream);
void RequestSerialHeading(Stream& dataStream);
void RequestSerialVelocity(Stream& dataStream);
void BlinkLEDCount(uint16_t count, uint16_t onTime = 100, uint16_t offTime = 100);
void IndicateFailure();


// Pin definitions
const int irqPin =  4;          // Interrupt pin, 2 and 3 are the Arduino's ext int pins, SAMD uses pin 4
const int ledPin = LED_BUILTIN; // Set up pin 13 LED for toggling

// Slave I2C connection on SERCOM0 (A3=SDA, A4=SCL)
TwoWire slaveI2C = TwoWire(&sercom0, A3, A4);

// The all-important MPU-9250 IMU
MPU9250 imu;

// IMU sensor data values
Vector3F acc0;                  // The accelerometer reading from the previous iteration
Vector3F velocity;              // Integrated velocity vector (m/s)
EulerAngles eulerAngles;        // Euler angles (yaw, pitch, roll)
float heading;                  // Compass heading

// Indicates which values should be output in continuous mode
bool modeContinuous = false;
bool accelContinuous = false;
bool gyroContinuous = false;
bool magContinuous = false;
bool eulerContinuous = false;
bool headingContinuous = false;
bool velocityContinuous = false;

// Indicates if outputting raw or scaled values
bool accelRaw = false;
bool gyroRaw = false;
bool magRaw = false;

// Buffer to hold incoming messages
uint8_t messageBuffer[33];
volatile uint8_t messageLength = 0;
volatile bool messagePending = false;

// Buffer to hold outgoing responses
uint8_t responseBuffer[33];
volatile uint8_t responseLength = 0;
volatile bool responsePending = false;

// Housekeeping variables
bool     ready = false;         // true=the AHRS is ready to work, false=still initializing
uint32_t lastUpdateTime = 0;    // Time of previous iteration (for calculating integration interval, dt)
uint16_t frameCount;
uint32_t frameStartTime;


SafePrint irqLogger;


//******************************************************************************
// IRQ handler for recieving a message on Sercom0
//******************************************************************************
void SERCOM0_Handler()
{
    slaveI2C.onService();
}


void setup()
{
    ready = false;

    // Start ConsoleStream first thing so we can see diagnostics
    ConsoleStream.begin(115200);

    // Wait for SerialUSB to initialize (max 1 second since startup)
    while (!ConsoleStream && millis() < 1000);

    // Start master I2C interface
    Logger() << "RTL_IMU_Razor: begin I2C Master..." << endl;
    Wire.begin();
    Wire.setTimeout(50);
    //TWBR = 12;  // Sets 400 kbit/sec I2C speed

    // Configure LED pin
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW);

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
    Logger() << "RTL_IMU_Razor: begin I2C Slave..." << endl;
    slaveI2C.begin(RAZOR_IMU_ADDRESS);

    // Must reassign pins A3 & A4 to SERCOM functionality
    // BUT it has to be done after the begin() call!
    pinPeripheral(A3, PIO_SERCOM_ALT);      // A3=SDA
    pinPeripheral(A4, PIO_SERCOM_ALT);      // A4=SCL

    // Configure Slave interrupt handlers
    slaveI2C.onReceive(ReceiveI2C);         // interrupt handler for incoming messages
    slaveI2C.onRequest(RequestI2C);         // interrupt handler for data requests

    // Configure MPU-9250 interrupt pin, its set as active high, push-pull
    Logger() << "RTL_IMU_Razor: Configure IRQ pin..." << endl;
    pinMode(irqPin, INPUT);
    digitalWrite(irqPin, LOW);

    // Verify connectivity with MPU-9250
    Logger() << "RTL_IMU_Razor: Test MPU-9250 connectivity... ";

    if (!imu.TestConnection(ConsoleStream))
    {
        Logger() << "Unable to connect to MPU-9250, Aborting..." << endl;

        // Loop forever if communication doesn't happen
        while (true) IndicateFailure();
    }

    Logger() << "Confirmed." << endl 
             << endl;

    // Results of gyro and accelerometer self test
    float selfTestResults[6];
    float magBias[3];
    float magSens[3];

    // Perform self-test, self-calibration, and initialization of MPU-9250
    Logger() << "RTL_IMU_Razor: Performing MPU-9250 self test..." << endl;
    imu.SelfTest(selfTestResults);

    Logger() << "MPU-9250 self-test results:" << endl;
    Logger() << "\tAccelerometer X-axis trim within " << _FLOAT(selfTestResults[0],1) << "% of factory value" << endl;
    Logger() << "\tAccelerometer Y-axis trim within " << _FLOAT(selfTestResults[1],1) << "% of factory value" << endl;
    Logger() << "\tAccelerometer Z-axis trim within " << _FLOAT(selfTestResults[2],1) << "% of factory value" << endl;
    Logger() << "\tGyroscope X-axis trim within "     << _FLOAT(selfTestResults[3],1) << "% of factory value" << endl;
    Logger() << "\tGyroscope Y-axis trim within "     << _FLOAT(selfTestResults[4],1) << "% of factory value" << endl;
    Logger() << "\tGyroscope Z-axis trim within "     << _FLOAT(selfTestResults[5],1) << "% of factory value" << endl
             << endl;

    Logger() << "RTL_IMU_Razor: Performing calibration..." << endl;
    imu.Calibrate(ConsoleStream);
    //imu.CalibrateFine(ConsoleStream);
    imu.GetMagCalibration(magBias, magSens);

    Logger() << "Magnetometer calibration values: " << endl;
    Logger() << "\tX-Axis sensitivity adjustment value " << _FLOAT(magSens[0], 2) << endl;
    Logger() << "\tY-Axis sensitivity adjustment value " << _FLOAT(magSens[1], 2) << endl;
    Logger() << "\tZ-Axis sensitivity adjustment value " << _FLOAT(magSens[2], 2) << endl
             << endl;

    Logger() << "RTL_IMU_Razor: Starting IMU..." << endl;
    imu.Begin(ConsoleStream);

    Logger() << endl;
    Logger() << "MPU-9250 is online..." << endl;

    frameCount = 0;
    frameStartTime = micros();

    ready = true;
}


void loop()
{
    auto now = micros();
    auto dt = now - lastUpdateTime;

    if (dt >= UpdateInterval)
    {
        imu.Update();

        // Get new scaled state vectors
        auto accl = imu.GetAccel();
        auto gyro = imu.GetGyro();
        auto mag = imu.GetMag();
        auto deltaT = dt / 1000000.0f;  // Time elapsed since last update

        //MadgwickQuaternionUpdate(accel.x, accel.y, accel.z, gyro.x*PI/180.0f, gyro.y*PI/180.0f, gyro.z*PI/180.0f, mag.y, mag.x, -mag.z, deltaT);
        MahonyQuaternionUpdate(accl, gyro, mag, deltaT);

        auto newEulerAngles = UpdateEulerAngles();
        auto acc1 = accl;
        auto dv = (acc1 + acc0) * (0.5f * ONE_G) * deltaT;  // Integrate velocity delta-v

        noInterrupts();
        eulerAngles = newEulerAngles;
        heading = imu.GetHeading();
        velocity += dv;
        interrupts();

        acc0 = acc1;
        lastUpdateTime = now;
        frameCount++;
    }
    // Don't allow other work if we are within 500us updating the IMU
    else if (dt < (UpdateInterval - 500))
    {
        irqLogger.flush();

        // Check for and process incoming serial requests
        if (DataStream.available())
        {
            ReceiveSerial(DataStream);
            RequestSerial(DataStream);
        }

        dt = now - frameStartTime;

        if (dt > 2000000ul)
        {
            auto frameRate = frameCount * 1000000.0 / dt;

            Logger() << F("Frame rate=") << frameRate << endl;
            frameCount = 0;
            frameStartTime = now;
        }

        if (modeContinuous)
        {
            if (accelContinuous) RequestSerialAccel(DataStream, accelRaw);

            if (gyroContinuous) RequestSerialGyro(DataStream, gyroRaw);

            if (magContinuous) RequestSerialMag(DataStream, magRaw);

            if (eulerContinuous) RequestSerialEuler(DataStream);

            if (headingContinuous) RequestSerialHeading(DataStream);

            if (velocityContinuous) RequestSerialVelocity(DataStream);

            // Toggle LED to indicate continuous mode active
            digitalWrite(ledPin, !digitalRead(ledPin));
        }
    }
}


//void DumpLastCommand()
//{
//    if (commandsToLog == 0) return;
//
//    uint8_t buffer[sizeof commandBuffer];
//    auto count = 0;
//
//    ZeroMemory(buffer, sizeof buffer);
//
//    noInterrupts();
//    CopyMemory(buffer, commandBuffer, sizeof commandLength);
//    count = commandsToLog;
//    commandsToLog = 0;
//    interrupts();
//
//    if (count > 1) Logger() << F("Unable to log ") << (count-1) << F(" previous command(s).") << endl;
//
//    if (count > 0) Logger() << F("Last Command =") << ToHex(buffer, sizeof buffer) << endl;
//}
//
//
//void DumpLastResponse()
//{
//    if (responsesToLog == 0) return;
//
//    uint8_t buffer[sizeof responseBuffer];
//    auto count = 0;
//
//    ZeroMemory(buffer, sizeof buffer);
//
//    noInterrupts();
//    CopyMemory(buffer, responseBuffer, responseLength);
//    count = responsesToLog;
//    responsesToLog = 0;
//    interrupts();
//
//    if (count > 1) Logger() << F("Unable to log ") << (count - 1) << F(" previous response(s).") << endl;
//
//    if (count > 0) Logger() << F("Last Response=") << ToHex(buffer, sizeof buffer) << endl;
//}


//****************************************************************************
// Interrupt handler for recieving a message on slave I2C connection
//****************************************************************************
static void ReceiveI2C(int dataLength)
{
    for (messageLength = 0; slaveI2C.available(); )
    {
        uint8_t c = slaveI2C.read();

        if (messageLength < sizeof(messageBuffer))  messageBuffer[messageLength++] = c;
    }
    
    responsePending = (messageLength > 0) ? ProcessI2CMessage() : false;
}


//****************************************************************************
// Interrupt handler for responding to a request on slave I2C connection
//****************************************************************************
static void RequestI2C()
{
    // Check for and process incoming I2C commands
    if (responsePending)
    {
        slaveI2C.write(responseBuffer, responseLength);
        responsePending = false;
    }
}


bool ProcessI2CMessage()
{
    responseLength = 0;

    if (DEBUG_I2C) irqLogger << F("Message Received[") << messageLength << F("]: ") << ToHex(messageBuffer, messageLength) << endl;

    switch (messageBuffer[0])
    {
        case REG_ID:
        {
            responseBuffer[0] = RAZOR_IMU_ID;
            responseLength = 1;
        }
        break;

        case REG_IS_READY:
        {
            responseBuffer[0] = (ready ? 1 : 0);
            responseLength = 1;
        }
        break;

        case REG_ACCEL:
        {
            auto data = imu.GetAccel();
            responseLength = sizeof data;
            CopyMemory(responseBuffer, (uint8_t*)&data, responseLength);
        }
        break;

        case REG_GYRO:
        {
            auto data = imu.GetGyro();
            responseLength = sizeof data;
            CopyMemory(responseBuffer, (uint8_t*)&data, responseLength);
        }
        break;

        case REG_MAG:
        {
            auto data = imu.GetMag();
            responseLength = sizeof data;
            CopyMemory(responseBuffer, (uint8_t*)&data, responseLength);
        }
        break;

        case REG_ACCEL_RAW:
        {
            auto data = imu.GetAccelRaw();
            responseLength = sizeof data;
            CopyMemory(responseBuffer, (uint8_t*)&data, responseLength);
        }
        break;

        case REG_GYRO_RAW:
        {
            auto data = imu.GetGyroRaw();
            responseLength = sizeof data;
            CopyMemory(responseBuffer, (uint8_t*)&data, responseLength);
        }
        break;

        case REG_MAG_RAW:
        {
            auto data = imu.GetMagRaw();
            responseLength = sizeof data;
            CopyMemory(responseBuffer, (uint8_t*)&data, responseLength);
        }
        break;

        case REG_EULER:
        {
            responseLength = sizeof eulerAngles;
            CopyMemory(responseBuffer, (uint8_t*)&eulerAngles, responseLength);
        }
        break;

        case REG_HEADING:
        {
            responseLength = sizeof heading;
            CopyMemory(responseBuffer, (uint8_t*)&heading, responseLength);
        }
        break;

        case REG_VELOCITY:
        {
            responseLength = sizeof velocity;
            CopyMemory(responseBuffer, (uint8_t*)&velocity, responseLength);
        }
        break;

        case REG_MAGBIAS_X:
        {
            auto data = *(int16_t*)&messageBuffer[1];
            imu.SetMagBiasX(data);
        }
        break;

        case REG_MAGBIAS_Y:
        {
            auto data = *(int16_t*)&messageBuffer[1];
            imu.SetMagBiasY(data);
        }
        break;

        case REG_MAGBIAS_Z:
        {
            auto data = *(int16_t*)&messageBuffer[1];
            imu.SetMagBiasZ(data);
        }
        break;
    }

    if (DEBUG_I2C && responseLength > 0) irqLogger << F("Response to send[") << responseLength << F("]: ") << ToHex(responseBuffer, responseLength) << endl;

    return (responseLength > 0);
}


//****************************************************************************
// Handler for recieving a message on serial connection
//****************************************************************************
void ReceiveSerial(Stream& dataStream)
{
    if (!dataStream.available()) return;

    auto bytesRead = dataStream.readBytesUntil('\n', messageBuffer, sizeof(messageBuffer)-1);

    messageBuffer[bytesRead] = 0;
}


//****************************************************************************
// Handler for responding to a request on serial connection
//****************************************************************************
void RequestSerial(Stream& dataStream)
{
    if (messageBuffer[0] == 0) return;


    if (messageBuffer[0] == PREFIX_CMD)
    {
        auto command = (char)messageBuffer[1];
        auto param1 = (char)messageBuffer[2];
        auto param2 = (char)messageBuffer[3];

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
            case CMD_ACCEL_RAW:
                RequestSerialAccel(dataStream, command == CMD_ACCEL_RAW);
                break;

            // Send gyro rates vector
            case CMD_GYRO:
            case CMD_GYRO_RAW:
                RequestSerialGyro(dataStream, command == CMD_GYRO_RAW);
                break;

            // Send magnetometer vector
            case CMD_MAG:
            case CMD_MAG_RAW:
                RequestSerialMag(dataStream, command == CMD_MAG_RAW);
                break;

            // Send Euler angles
            case CMD_EULER:
                RequestSerialEuler(dataStream);
                break;

            // Send compass heading
            case CMD_HEADING:
                RequestSerialHeading(dataStream);
                break;

            case CMD_VELOCITY:
                RequestSerialVelocity(dataStream);
                break;

            case CMD_ZERO:
                velocity.x = velocity.y = velocity.z = 0.0;
                break;

            case CMD_CONTINUOS:
                ProcessContinuousModeCommand(dataStream, command, param1, param2);
                break;

            case CMD_RESET:
                setup();
                break;

            case CMD_DEBUG_I2C:
                DEBUG_I2C = (param1 == '1');
                break;

            default:
                dataStream << PREFIX_ERR << _HEX(ERR_CMD_UNRECOGNIZED) << endl;
                break;
        }
    }
    else
    {
        dataStream << PREFIX_ERR << _HEX(ERR_BAD_CMD_STRING) << endl;
    }

    // Clears command buffer
    messageBuffer[0] = 0;
}


void ProcessContinuousModeCommand(Stream& dataStream, const char command, const char param1, const char param2)
{
    switch (param1)
    {
    case '1':
    case '0':
        modeContinuous = param1 == '1';
        dataStream << PREFIX_RESPONSE << command << ' ' << modeContinuous << endl;
        break;
    
    case '9':
    {
        auto enableAll = param2 == '1';

        accelContinuous = enableAll;
        gyroContinuous = enableAll;
        magContinuous = enableAll;
        eulerContinuous = enableAll;
        headingContinuous = enableAll;
        velocityContinuous = enableAll;
        dataStream << PREFIX_RESPONSE << command << ' ' << enableAll << endl;
        break;
    }
    
    case CMD_ACCEL_RAW:
    case CMD_ACCEL:
        accelRaw = param1 == CMD_ACCEL_RAW;
        accelContinuous = param2 == '1';
        dataStream << PREFIX_RESPONSE << command << param1 << ' ' << accelContinuous << endl;
        break;
    
    case CMD_GYRO:
    case CMD_GYRO_RAW:
        gyroRaw = param1 == CMD_GYRO_RAW;
        gyroContinuous = param2 == '1';
        dataStream << PREFIX_RESPONSE << command << param1 << ' ' << gyroContinuous << endl;
        break;
    
    case CMD_MAG:
    case CMD_MAG_RAW:
        magRaw = param1 == CMD_MAG_RAW;
        magContinuous = param2 == '1';
        dataStream << PREFIX_RESPONSE << command << param1 << ' ' << magContinuous << endl;
        break;
    
    case CMD_EULER:
        eulerContinuous = param2 == '1';
        dataStream << PREFIX_RESPONSE << command << param1 << ' ' << eulerContinuous << endl;
        break;
    
    case CMD_HEADING:
        headingContinuous = param2 == '1';
        dataStream << PREFIX_RESPONSE << command << param1 << ' ' << headingContinuous << endl;
        break;

    case CMD_VELOCITY:
        velocityContinuous = param2 == '1';
        dataStream << PREFIX_RESPONSE << command << param1 << ' ' << velocityContinuous << endl;
        break;
    }
}


void RequestSerialID(Stream& dataStream)
{
    dataStream << PREFIX_RESPONSE << CMD_ID << ' ' << ' ' << _HEX(RAZOR_IMU_ID) << endl;
}


void RequestSerialIsReady(Stream& dataStream)
{
    dataStream << PREFIX_RESPONSE << CMD_IS_READY << ' ' << ' ' << (ready ? '1' : '0') << endl;
}


void RequestSerialAccel(Stream& dataStream, bool modeRaw)
{
    if (modeRaw)
        dataStream << PREFIX_RESPONSE << CMD_ACCEL_RAW << ' ' << millis() << ' ' << imu.GetAccelRaw().x << ' ' << imu.GetAccelRaw().y << ' ' << imu.GetAccelRaw().z << endl;
    else
        dataStream << PREFIX_RESPONSE << CMD_ACCEL << ' ' << millis() << ' ' << _FLOAT(imu.GetAccel().x, 4) << ' ' << _FLOAT(imu.GetAccel().y, 4) << ' ' << _FLOAT(imu.GetAccel().z, 4) << endl;
}


void RequestSerialGyro(Stream& dataStream, bool modeRaw)
{
    if (modeRaw)
        dataStream << PREFIX_RESPONSE << CMD_GYRO_RAW << ' ' << millis() << ' ' << imu.GetGyroRaw().x << ' ' << imu.GetGyroRaw().y << ' ' << imu.GetGyroRaw().z << endl;
    else
        dataStream << PREFIX_RESPONSE << CMD_GYRO << ' ' << millis() << ' ' << _FLOAT(imu.GetGyro().x, 4) << ' ' << _FLOAT(imu.GetGyro().y, 4) << ' ' << _FLOAT(imu.GetGyro().z, 4) << endl;
}


void RequestSerialMag(Stream& dataStream, bool modeRaw)
{
    if (modeRaw)
        dataStream << PREFIX_RESPONSE << CMD_MAG_RAW << ' ' << millis() << ' ' << imu.GetMagRaw().x << ' ' << imu.GetMagRaw().y << ' ' << imu.GetMagRaw().z << endl;
    else
        dataStream << PREFIX_RESPONSE << CMD_MAG << ' ' << millis() << ' ' << _FLOAT(imu.GetMag().x, 4) << ' ' << _FLOAT(imu.GetMag().y, 4) << ' ' << _FLOAT(imu.GetMag().z, 4) << endl;
}


void RequestSerialEuler(Stream& dataStream)
{
    dataStream << PREFIX_RESPONSE << CMD_EULER << ' ' << millis() << ' ' << _FLOAT(eulerAngles.Yaw, 2) << ' ' << _FLOAT(eulerAngles.Pitch, 2) << ' ' << _FLOAT(eulerAngles.Roll, 2) << endl;
}


void RequestSerialHeading(Stream& dataStream)
{
    dataStream << PREFIX_RESPONSE << CMD_HEADING << ' ' << millis() << ' ' << _FLOAT(heading, 2) << endl;
}


void RequestSerialVelocity(Stream& dataStream)
{
    dataStream << PREFIX_RESPONSE << CMD_VELOCITY << ' ' << millis() << ',' << _FLOAT(velocity.x, 4) << ',' << _FLOAT(velocity.y, 4) << ',' << _FLOAT(velocity.z, 4) << endl;
}


void BlinkLEDCount(uint16_t count, uint16_t onTime, uint16_t offTime)
{
    for (int i = 0; i < count; i++)
    {
      digitalWrite(ledPin, HIGH);
      delay(onTime);
      digitalWrite(ledPin, LOW);
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

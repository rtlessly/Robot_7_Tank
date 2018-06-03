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

All commands begin with "#" and is followed by a letter indicating the requested
data. The letters are (must be upper case):
    - a : Return 3-axis acceleration vector in g's
    - A : Return 3-axis dynamic acceleration vector in g's (gravity component removed)
    - g : Return 3-axis gyroscope rate vector in radians/sec
    - G : Return 3-axis integrated gyroscope gimbal angles in degrees
    - m : Return 3-axis magnetometer vector in milliGuass
    - V : Return 3-axis integrated velocity vector in meters/sec
    - E : Return Euler angles as yaw, pitch, roll in degrees
    - I : Returns the Firmware ID (0x50)

For example, to request Euler angles, send the command "QE" via either I2C or
the serial port. 

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
 */
#include <Arduino.h>
#include <wiring_private.h> // To get pinPeripheral() function
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


// Command Codes
const char CMD_PREFIX    = '#';
const char CMD_ID        = 'I';
const char CMD_ACCEL     = 'a';
const char CMD_DYN_ACCEL = 'A';
const char CMD_GYRO      = 'g';
const char CMD_GIMBAL    = 'G';
const char CMD_MAG       = 'm';
const char CMD_VELOCITY  = 'V';
const char CMD_EULER     = 'E';
const char CMD_CONTINUOS = 'c';


// Forward function declarations
void BlinkLEDCount(uint16_t count, uint16_t onTime=100, uint16_t offTime=100);
void IndicateFailure();
void ReceiveSerial(Stream& dataStream);


// Pin definitions
int intPin =  4;            // Interrupt pin, 2 and 3 are the Arduino's ext int pins, SAMD uses pin 4
int HW_LED_PIN = 13;        // Set up pin 13 led for toggling

// The all-important MPU-9250 IMU
MPU9250 imu;

// IMU sensor data values
ScaledData accel;           // Scaled accelerometer vector (g)
ScaledData gyro;            // Scaled gyro rates (radians/sec)
ScaledData mag;             // Scaled magnetometer vector (milliGaus)
ScaledData dynAccel;        // Dynamic acceleration (gravity component removed)
ScaledData velocity;        // Integrated velocity vector (m/s)
ScaledData gimbal;          // Integrated gimbal angles (degrees)

// Housekeeping variables
uint32_t lastUpdate = 0;    // Time of previous iteration - used to calculate integration interval

bool modeContinuous = false;
bool velocityContinuous = true;
bool eulerContinuous = true;
bool accelContinuous = true;
bool gyroContinuous = true;
bool magContinuous = true;

// Slave I2C connection on SERCOM0 (A3=SDA, A4=SCL)
TwoWire  slaveI2C = TwoWire(&sercom0, A3, A4);

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

    ConsoleStream << "Basic AHRS: In setup()..." << endl;

    // Start I2C interface, aka Two-Wire Interface (TWI)
    ConsoleStream << "Basic AHRS: begin I2C Master..." << endl;
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
    ConsoleStream << "Basic AHRS: begin I2C Slave..." << endl;
    slaveI2C.begin(RAZOR_IMU_ADDRESS);

    // Must reassign pins A3 & A4 to SERCOM functionality
    // BUT it has to be done after the begin() call!
    pinPeripheral(A3, PIO_SERCOM_ALT);      // A3=SDA
    pinPeripheral(A4, PIO_SERCOM_ALT);      // A4=SCL

    // Configure Slave interrupt handlers
    slaveI2C.onReceive(ReceiveI2C);         // interrupt handler for incoming messages
    slaveI2C.onRequest(RequestI2C);         // interrupt handler for data requests

    // Configure MPU-9250 interrupt pin, its set as active high, push-pull
    ConsoleStream << "Basic AHRS: Configure pins..." << endl;
    pinMode(intPin, INPUT);
    digitalWrite(intPin, LOW);

    // Configure LED pin
    pinMode(HW_LED_PIN, OUTPUT);
    digitalWrite(HW_LED_PIN, LOW);

    // Verify connectivity with MPU-9250
    ConsoleStream << "Basic AHRS: Test MPU-9250 connectivity..." << endl;
    if (!imu.TestConnection(ConsoleStream))
    {
        ConsoleStream << "Unable to connect to MPU-9250, Aborting..." << endl;

        // Loop forever if communication doesn't happen
        while (true) IndicateFailure();
    }

    // Results of gyro and accelerometer self test
    float selfTestResults[6];

    // Perform self-test, self-calibration, and initialization of MPU-9250
    ConsoleStream << "Basic AHRS: Performing self test..." << endl;
    imu.SelfTest(selfTestResults);
    ConsoleStream << "Basic AHRS: Performing calibration..." << endl;
    imu.Calibrate(ConsoleStream);
    ConsoleStream << "Basic AHRS: Starting IMU..." << endl;
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
}


void loop()
{
    const float one_g = 9.80665; // Converts g's to m/s^2

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

    //MadgwickQuaternionUpdate(accel.X, accel.Y, accel.Z, gyro.X*PI/180.0f, gyro.Y*PI/180.0f, gyro.Z*PI/180.0f, mag.Y, mag.X, -mag.Z, deltaT);
    //MahonyQuaternionUpdate(accel.X, accel.Y, accel.Z, gyro.X*PI/180.0f, gyro.Y*PI/180.0f, gyro.Z*PI/180.0f, mag.X, mag.Y, mag.Z, deltaT);

    // NOTE: The Mahony filter returns an updated gyro rate vector to correct for gyro drift
    gyro = MahonyQuaternionUpdate(accel, gyro, mag, deltaT);
    UpdateEulerAngles();

    // Integrate velocity
    auto a = imu.GetDynamicAccel();
    auto f = 0.5f * one_g * deltaT;

    velocity.X += (a.X + dynAccel.X) * f;
    velocity.Y += (a.Y + dynAccel.Y) * f;
    velocity.Z += (a.Z + dynAccel.Z) * f;
    dynAccel = a;

    // Integrate gyro rates to update gimbal angles in degrees
    f = 0.5 * (180.0 / PI) * deltaT;
    gimbal.X += (gyro.X + prevGyro.X) * f;
    gimbal.Y += (gyro.Y + prevGyro.Y) * f;
    gimbal.Z += (gyro.Z + prevGyro.Z) * f;

    // Check for and process incoming serial requests
    if (DataStream.available())
    {
        ReceiveSerial(DataStream);
        RequestSerial(DataStream);
    }

    if (modeContinuous)
    {
        if (accelContinuous) DataStream << CMD_ACCEL << ' ' << _FLOAT(accel.X, 2) << ' ' << _FLOAT(accel.Y, 2) << ' ' << _FLOAT(accel.Z, 2) << endl;

        if (gyroContinuous) DataStream << CMD_GYRO << ' ' << _FLOAT(gyro.X, 2) << ' ' << _FLOAT(gyro.Y, 2) << ' ' << _FLOAT(gyro.Z, 2) << endl;
        
        if (magContinuous) DataStream << CMD_MAG << ' ' << _FLOAT(mag.X, 2) << ' ' << _FLOAT(mag.Y, 2) << ' ' << _FLOAT(mag.Z, 2) << endl;

        if (eulerContinuous) DataStream << CMD_EULER << ' ' << _FLOAT(yaw, 2) << ' ' << _FLOAT(pitch, 2) << ' ' << _FLOAT(roll, 2) << endl;

        if (velocityContinuous) DataStream << CMD_VELOCITY << ' ' << _FLOAT(velocity.X, 2) << ' ' << _FLOAT(velocity.Y, 2) << ' ' << _FLOAT(velocity.Z, 2) << endl;

        // Toggle LED to indicate communication occurred
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
                slaveI2C.write(RAZOR_IMU__ID);
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
        switch (commandBuffer[1])
        {
            case CMD_ID:
                DataStream << CMD_ID << ' ' << _HEX(RAZOR_IMU__ID) << endl;
                break;

            // Send acceleration vector
            case CMD_ACCEL:
                DataStream << CMD_ACCEL << ' ' << _FLOAT(accel.X, 2) << ' ' << _FLOAT(accel.Y, 2) << ' ' << _FLOAT(accel.Z, 2) << endl;
                break;

            // Send dynamic acceleration vector
            case CMD_DYN_ACCEL:
                DataStream << CMD_DYN_ACCEL << ' ' << _FLOAT(dynAccel.X, 2) << ' ' << _FLOAT(dynAccel.Y, 2) << ' ' << _FLOAT(dynAccel.Z, 2) << endl;
                break;

            // Send gyro rates vector
            case CMD_GYRO:
                DataStream << CMD_GYRO << ' ' << _FLOAT(gyro.X, 2) << ' ' << _FLOAT(gyro.Y, 2) << ' ' << _FLOAT(gyro.Z, 2) << endl;
                break;

            // Send gimbal angles vector
            case CMD_GIMBAL:
                DataStream << CMD_GIMBAL << ' ' << _FLOAT(gimbal.X, 2) << ' ' << _FLOAT(gimbal.Y, 2) << ' ' << _FLOAT(gimbal.Z, 2) << endl;
                break;

            // Send magnetometer vector
            case CMD_MAG:
                DataStream << CMD_MAG << ' ' << _FLOAT(mag.X, 2) << ' ' << _FLOAT(mag.Y, 2) << ' ' << _FLOAT(mag.Z, 2) << endl;
                break;

            // Send velocity vector
            case CMD_VELOCITY:
                DataStream << CMD_VELOCITY << ' ' << _FLOAT(velocity.X, 2) << ' ' << _FLOAT(velocity.Y, 2) << ' ' << _FLOAT(velocity.Z, 2) << endl;
                break;

            // Send Euler angles
            case CMD_EULER:
                DataStream << CMD_EULER << ' ' << _FLOAT(yaw, 2) << ' ' << _FLOAT(pitch, 2) << ' ' << _FLOAT(roll, 2) << endl;
                break;

            case CMD_CONTINUOS:
            {
                auto param = commandBuffer[2];

                switch (param)
                {
                    case '1':
                        modeContinuous = true;
                        DataStream << CMD_CONTINUOS << ' ' << "ON" << endl;
                        break;
                    case '0':
                        modeContinuous = false;
                        DataStream << CMD_CONTINUOS << ' ' << "OFF" << endl;
                        break;
                    case 'a':
                        accelContinuous = commandBuffer[3] == '1';
                        DataStream << CMD_CONTINUOS << " accel " << (accelContinuous ? "ON" : "OFF") << endl;
                        break;
                    case 'g':
                        gyroContinuous = commandBuffer[3] == '1';
                        DataStream << CMD_CONTINUOS << " gyro " << (gyroContinuous ? "ON" : "OFF") << endl;
                        break;
                    case 'm':
                        magContinuous = commandBuffer[3] == '1';
                        DataStream << CMD_CONTINUOS << " mag " << (magContinuous ? "ON" : "OFF") << endl;
                        break;
                    case 'E':
                        eulerContinuous = commandBuffer[3] == '1';
                        DataStream << CMD_CONTINUOS << " euler " << (eulerContinuous ? "ON" : "OFF") << endl;
                        break;
                    case 'V':
                        velocityContinuous = commandBuffer[3] == '1';
                        DataStream << CMD_CONTINUOS << " velocity " << (velocityContinuous ? "ON" : "OFF") << endl;
                        break;
                }
                break;
            }

            default:
                DataStream << "0xFE" << endl;   // 0xFE=Unrecognized command code
                break;
        }
    }
    else
    {
        DataStream << "0xFF" << endl;           // 0xFF=Unrecognized command string
    }

    // Clears command buffer
    commandBuffer[0] = 0;
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

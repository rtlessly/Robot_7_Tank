/*******************************************************************************
Name:       RTL_IR_RemoteDecoder.ino
Created:    8/25/2018 4:04:59 PM
Author:     RTLessly-Laptop\R. T. Lessly

Arduino Sketch for dedicated IR Remote decoder. Default behavior is to decode
the NEC IR remote protocol.

Receiving and decoding IR remote signals is a task that requires critical timing. As
such, this sketch is intended to run on an Arduino dedicated solely to this purpose. 
An Arduino Nano is a good choice for this.

IR Signals are received via a simple IR sensor, which is expected to be connected
to digital pin 2 by default (can be changed via the constant IR_SENSOR_PIN). As
signals are received and decoded, the corresponding commands are cached in a
circular buffer until they are retrieved by an external device via the I2C interface.
A maximum of 10 commands can be buffered. If the buffer fills up then the oldest
command in the buffer is replaced with the lastest decoded command.

This sketch operates as an I2C slave at address 0x45 by default (can be changed via
constant IR_REMOTE_I2C_ADDRESS in RTL_IR_RemoteDecoder.h). Simply performing an I2C 
request operation to this address will return the next command in the buffer. If no
command is available then a command with a type code of 0 is returned.

A command returned by the I2C interface is defined by the IRRemoteCommand struct
(declared in RTL_IR_RemoteDecoder.h), which is 6 bytes long and consists of 3 fields:

Field     DataType  Description
--------  --------  ------------------------------------------------------------
Type       uint8    Command type: 0=no command; 1=normal command; 3=repeated command.
Protocol   uint8    Indicates the command protocol. See enum decode_type_t in IRremote.h
                    for possible values. Currently only the NEC protocol is 
                    implemented, so this value will always be 3.
Code       uint32   The actual command code. The meaning of the command code is
                    dependent on the protocol. See RTL_IR_CommandCodes.h for NEC
                    command codes.
******************************************************************************/
#define DEBUG 0

#include <Arduino.h>
#include <Wire.h>
#include <RTL_Stdlib.h>
#include <RTL_Debug.h>
#include <IRRemoteReceiver.h>
#include "RTL_IR_RemoteDecoder.h"


const int IR_SENSOR_PIN = 2;        // Digital pin connected to the IR sensor
const int IR_CMD_BUFFER_LEN = 10;   // Command buffer length

static IRRemoteReceiver irReceiver(IR_SENSOR_PIN, LED_BUILTIN);

static IRRemoteCommand buffer[IR_CMD_BUFFER_LEN];
static byte bufferHead = 0;
static byte bufferTail = 0;
static byte bufferCount = 0;

static uint32_t lastCommandCode = IR_NONE;


void setup()
{
    Serial.begin(115200);
    Wire.begin(IR_REMOTE_I2C_ADDRESS);
    irReceiver.Begin();

    // Configure Slave interrupt handlers
    Wire.onReceive(ReceiveI2C);         // interrupt handler for incoming messages
    Wire.onRequest(RequestI2C);         // interrupt handler for data requests

    TRACE(Logger() << F("IRReceiver ready") << endl);
}


void loop()
{
    decode_results results;

    // check to see if the receiver has received a command
    if (!irReceiver.decode(&results)) return;

    TRACE(Logger() << F("Decoded=") << _HEX(results.value) << endl);

    // buffer the decoded command
    if (!results.overflow)
    {
        IRRemoteCommand command;

        command.Protocol = results.decode_type;

        if (results.value != IR_REPEAT)
        {
            command.Type = IRRemoteCommandType::Normal;
            command.Code = results.value;
            lastCommandCode = results.value;
        }
        else
        {
            command.Type = IRRemoteCommandType::Repeat;
            command.Code = lastCommandCode;
        }

        // BEGIN CRITICAL SECTION while updating command buffer
        // This critical section is necessary to prevent any incoming I2C 
        // requests from interfering with the buffer update. It is possible
        // that an I2C request could try to read the buffer while this
        // update is occurring, which could cause the buffer head and tail
        // to get out of sync.
        // This may be overkill since I think the only really critical line
        // of code is in the else clause below. However, a buffer update
        // should be treated as an atomic operation, and a 16MHz UNO/Nano 
        // has ample performance for this, so it doesn't hurt anything to 
        // mark the entire buffer update operation as a critical section. 
        // The whole critical section should be no more than 5 microseconds.
        noInterrupts(); 
        {
            buffer[bufferTail] = command;
            bufferTail = (bufferTail + 1) % IR_CMD_BUFFER_LEN;
    
            if (bufferCount < IR_CMD_BUFFER_LEN)
            {
                bufferCount++;
            }
            else
            {
                // Buffer was full and current head was overwritten, so bump head to next command in buffer
                bufferHead = (bufferHead + 1) % IR_CMD_BUFFER_LEN;
            }
        }
        interrupts(); // END CRITICAL SECTION
        
        TRACE(Logger() << F("bufferHead=") << bufferHead << F(", bufferTail=") << bufferTail << F(", bufferCount=") << bufferCount << endl);
    }

    // Ready to listen for another command
    irReceiver.resume();
}


//******************************************************************************
// Interrupt handler for recieving a message on slave I2C connection
//******************************************************************************
static void ReceiveI2C(int messageLength)
{
}


//******************************************************************************
// Interrupt handler for responding to a request on slave I2C connection
//******************************************************************************
static void RequestI2C()
{
    IRRemoteCommand command { 0, 0, 0 };

    // Critical section not needed here since interrupts already disabled inside IRQ handler
    if (bufferCount > 0)
    {
        command = buffer[bufferHead];
        bufferHead = (bufferHead + 1) % IR_CMD_BUFFER_LEN;
        bufferCount--;
    }

    Wire.write((uint8_t*)&command, sizeof(command));
}

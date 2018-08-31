#ifndef _RTL_IR_RemoteDecoder_h
#define _RTL_IR_RemoteDecoder_h

#include <IRCommandCodes.h>

#define IR_REMOTE_I2C_ADDRESS     ((byte)0x45)

enum IRRemoteCommandType
{
    None = 0,
    Normal = 1,
    Repeat = 3
};


struct IRRemoteCommand
{
    uint8_t  Type;          // Command type (see IRRemoteCommandType above)
    uint8_t  Protocol;      // Code for command protocol (3 = NEC)
    uint32_t Code;          // The actual command code
};

#endif

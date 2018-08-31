#ifndef _RTL_IR_RemoteDecoder_h_
#define _RTL_IR_RemoteDecoder_h_

#include "RTL_IR_CommandCodes.h"

#define IR_REMOTE_I2C_ADDRESS     ((byte)0x45)

enum IRRemoteCommandType
{
    None   = 0x00,
    Normal = 0x01,
    Repeat = 0x03,
    End    = 0x80
};


struct IRRemoteCommand
{
    uint8_t  Type;          // Command type (see IRRemoteCommandType above)
    uint8_t  Protocol;      // Code for command protocol (3 = NEC)
    uint32_t Code;          // The actual command code
};

#endif

#ifndef _RTL_IR_CommandCodes_h_
#define _RTL_IR_CommandCodes_h_


#define IR_NONE 0

//******************************************************************************
// IR Command Codes for NEC IR Remotes
//******************************************************************************
#define IR_CH_MINUS  0xFFA25D       // Channel minus (aka channel down)
#define IR_CH        0xFF629D       // Channel recall/display
#define IR_CH_PLUS   0xFFE21D       // Channel plus (aka channel up)

#define IR_PREV      0xFF22DD       // Previous track / rewind
#define IR_NEXT      0xFF02FD       // Next track / Fast forward
#define IR_PLAY      0xFFC23D       // Play/pause

#define IR_VOL_MINUS 0xFFE01F       // volume minus (aka volume down)
#define IR_VOL_PLUS  0xFFA857       // volume plus (aka volume up)
#define IR_VOL_EQ    0xFF906F       // volume equalizer

#define IR_100_PLUS  0xFF9867       // 100 + number (indicates following 2-digit number is in the range 100-199)
#define IR_200_PLUS  0xFFB04F       // 200 + number (indicates following 2-digit number is in the range 200-299)

#define IR_0         0xFF6897       // 0 digit
#define IR_1         0xFF30CF       // 1 digit
#define IR_2         0xFF18E7       // 2 digit
#define IR_3         0xFF7A85       // 3 digit
#define IR_4         0xFF10EF       // 4 digit
#define IR_5         0xFF38C7       // 5 digit
#define IR_6         0xFF5AA5       // 6 digit
#define IR_7         0xFF42BD       // 7 digit
#define IR_8         0xFF4AB5       // 8 digit
#define IR_9         0xFF52AD       // 9 digit

#define IR_REPEAT    0xFFFFFFFF     // Repeat previous command

#endif

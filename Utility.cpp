
#define DEBUG 0

#include "Robot_7_Tank.h"


void BlinkLEDCount(uint16_t count, uint16_t onTime, uint16_t offTime)
{
    for (int i = 0; i < count; i++)
    {
        wdt_reset();
        digitalWrite(LED_PIN, HIGH);
        delay(onTime);
        digitalWrite(LED_PIN, LOW);
        delay(offTime);
    }
}


void IndicateFailure(const __FlashStringHelper* msg)
{
    if (msg != nullptr) Logger() << msg << endl;

    // This flashes "SOS" in morse code
    BlinkLEDCount(3, 100, 150);
    delay(200);
    BlinkLEDCount(3, 300, 150);
    delay(200);
    BlinkLEDCount(3, 100, 150);
    delay(500);
}

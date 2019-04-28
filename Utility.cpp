
#define DEBUG 0

#include "Robot_7_Tank.h"


void BlinkLED(uint16_t onTime, uint16_t offTime)
{
    digitalWrite(LED_PIN, HIGH);
    delay(onTime);
    digitalWrite(LED_PIN, LOW);
    
    if (offTime > 0) delay(offTime);
}


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


void IndicateFailure(int step, const __FlashStringHelper* msg, bool loopForever)
{
    if (msg != nullptr) Logger() << msg << endl;

    do
    {
        BlinkLED(1000, 500);
        BlinkLEDCount(step, 50, 200);
        delay(1000);
    }
    while (loopForever);
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

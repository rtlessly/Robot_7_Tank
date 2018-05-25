#ifndef _ULTRASONIC_H_
#define _ULTRASONIC_H_

#include <stdint.h>

class Ultrasonic
{
  public:
    Ultrasonic(uint8_t triggerPin, uint8_t echoPin);
    
    float PingCentimeters();
    float MultiPing();
    
    float Distance;
    
  private:
    uint8_t _triggerPin;
    uint8_t _echoPin;
};

#endif

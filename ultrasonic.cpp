#include <Arduino.h>
#include "ultrasonic.h"

Ultrasonic::Ultrasonic(uint8_t triggerPin, uint8_t echoPin)
{
    _triggerPin = triggerPin;
    _echoPin = echoPin;
    Distance = 0;
    
    pinMode(_echoPin, INPUT);
    pinMode(_triggerPin, OUTPUT);
}


float Ultrasonic::PingCentimeters()
{
    /*------------------------------------------------------------------------
    To trigger a ping you have to cycle the trigger pin low for 2 microsceonds, 
    then high for 10 microseconds, and then low again.
    ------------------------------------------------------------------------*/
    // Start by holding trigger low for 2 microseconds
    digitalWrite(_triggerPin, LOW);
    delayMicroseconds(2);
    
    // Now hold trigger high for 10 microseconds
    digitalWrite(_triggerPin, HIGH);
    delayMicroseconds(10);
    
    // Finally, set trigger low again to start ping
    digitalWrite(_triggerPin, LOW);

    // Wait for echo pin to go low to indicate receipt of echo, with 3000us timeout.
    // pulseIn() returns the time the pin was in high state, in microseconds (or 0 if timeout)
    // Convert ping time to centimeters by dividing by 58 microseconds / 2 centimeters
    // (since ping time is for round trip = 2 x distance)
    Distance = pulseIn(_echoPin, HIGH, 3000) / 58.0;
    
    return Distance;
}

//***************************************************************************
// Take an ultrasonic sensor sample. 
// This method takes 10 samples and averages them, which is one way to reduce 
// the effect of noisy data.
//***************************************************************************
float Ultrasonic::MultiPing()
{
    auto sum = 0.0F;
    auto count = 0;

    for (auto i = 0; i < 10; i++)
    {
        auto sample = PingCentimeters();

        // throw out bad samples
        if (sample == 0) continue;
      
        sum += sample;
        count++;
    }
    
    return (count > 0) ? (sum / count) : 500;
}






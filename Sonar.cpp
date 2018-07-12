
#define DEBUG 0

#include "Robot_7_Tank.h"


//******************************************************************************
// Do one ultrasonic sensor ping.
//******************************************************************************
uint16_t Ping()
{
    uint16_t ping;

    // Keep retrying until we get a good ping (or the watchdog timer resets the board)
    while ((ping = sonar.PingCentimeters()) == PING_FAILED) delay(40);

    return ping;
}


//******************************************************************************
/// Use the ultrasonic sensor to scan from right to left. 
/// Return whichever direction has more open space (i.e., the direction in which 
/// the ultrasonic sensor pings the greatest distances). Return value is 'L' if
/// the left side appears more open, or 'R' if the right side appears more open.
/// A simple way to do this is to just sum the distances for each side. Whichever
/// side has the greater sum is probably more open.
/// A more sophisticated approach might be to do some sort of statistical analysis
///to detect outliers, compute the median, etc.
//******************************************************************************
SonarScanResults ScanForBetterDirection()
{
    static const int SCAN_ANGLE = 90;

    uint16_t ping;
    uint16_t leftSum = 0;
    uint16_t leftMax = 0;
    uint16_t rightSum = 0;
    uint16_t rightMax = 0;
    //uint16_t leftscan[SCAN_ANGLE];
    //uint16_t rightscan[SCAN_ANGLE];

    panServo.write(SERVO_BIAS);  // Start sonar at center position looking straight ahead
    delay(100);                  // Wait 100ms for servo to reach start position
    wdt_reset();

    //ZERO_ARRAY(leftscan);
    //ZERO_ARRAY(rightscan);

    // Scan distances from center to max right angle (right side cw)
    for (auto i = 1; i <= SCAN_ANGLE; i++)
    {
        panServo.write(SERVO_BIAS - i);
        delay(2);				// Delay 2ms for servo to move to next position
        //rightscan[i - 1] += Ping();
        ping = Ping();
        rightSum += ping;
        rightMax = max(rightMax, ping);
        wdt_reset();
    }

    // Scan distances from max right angle to center (right side ccw)
    for (auto i = SCAN_ANGLE; i >= 1; i--)
    {
        panServo.write(SERVO_BIAS - i);
        delay(2);				// Delay 2ms for servo to move to next position
        //rightscan[i - 1] += Ping();
        ping = Ping();
        rightSum += ping;
        rightMax = max(rightMax, ping);
        wdt_reset();
    }

    // Scan distances from center to max left (left side ccw)
    for (auto i = 1; i <= SCAN_ANGLE; i++)
    {
        panServo.write(SERVO_BIAS + i);
        delay(2);				// Delay 2ms for servo to move to next position
        //leftscan[i - 1] += Ping();
        ping = Ping();
        leftSum += ping;
        leftMax = max(leftMax, ping);
        wdt_reset();
    }

    // Scan the distance from max left angle to center (left side cw)
    for (auto i = SCAN_ANGLE; i >= 1; i--)
    {
        panServo.write(SERVO_BIAS + i);
        delay(2);				// Delay 2ms for servo to move to next position
        //leftscan[i - 1] += Ping();
        ping = Ping();
        leftSum += ping;
        leftMax = max(leftMax, ping);
        wdt_reset();
    }

    // All done scanning. At this point note that both sides have been scanned twice.
    // Recenter servo to point straight ahead again
    panServo.write(SERVO_BIAS);

    //for (auto i = 0; i < SCAN_ANGLE; i++)
    //{
    //    left += leftscan[i];
    //    right += rightscan[i];
    //}

    auto direction = (leftSum > rightSum) ? 'L' : 'R';

    TRACE(Logger() << F("ScanForBetterDirection, direction=") << direction
                   << F(", leftSum") << leftSum << F(", rightSum=") << rightSum
                   << F(", leftMax=") << leftMax << F(", rightMax=") << rightMax
                   << endl);

    return SonarScanResults{ direction, leftSum, rightSum, leftMax, rightMax };
}


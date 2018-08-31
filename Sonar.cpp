
#define DEBUG 0

#include "Robot_7_Tank.h"


// Sonar pan servo center position bias. This is the value you have to send to
// the servo to set it to the centered position. Ideally, this should be 90 since
// the Arduino servo library takes angle values from 0 to 180 degrees. However,
// this value may need to be tweaked slightly depending on the physical characteristics
// of the servo motor and how accurately the ultrasonic sensor can be mounted and
// aligned to the true center position of the servo shaft. 
const int SERVO_BIAS = 93;

int16_t sonarAngle = 0;


//******************************************************************************
// Pan the sonar servo to the desired angle. The angle should be in the range
// +/-90 degrees, with 0 degrees being the centered position.
//******************************************************************************
void PanSonar(int angle)
{
    panServo.write(SERVO_BIAS + angle);
    sonarAngle = angle;
}


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
// Do one ultrasonic sensor ping at specified angle. Also provides low-pass filtering.
//******************************************************************************
float PingAt(int16_t angle, float* prevPing=nullptr)
{
    uint32_t servoDelay = 5 * abs(angle - sonarAngle);  // Assume 5ms per degree

    PanSonar(angle);        // Move to ping position
    delay(servoDelay);      // Delay for servo to move to next position
    wdt_reset();

    float ping = Ping();

    // Apply lo-pass filter if previous ping value was provided
    if (prevPing != nullptr) ping = 0.8*(*prevPing) + 0.2*ping;

    return ping;
}


//******************************************************************************
/// Use the ultrasonic sensor to scan from right to left to determine what direction
/// is most open and thus the best direction to move. Returns the value 'L' if
/// the left side appears more open, or 'R' if the right side appears more open.
/// A simple way to do this is to scan each side in 1 degree increments and then
/// sum the individual scan distances for each side. This essentially integrates 
/// the scan distances to find a proxy for the open area to the left and right. 
/// Whichever side has the greater sum is probably more open. This function uses
/// a low-pass filter to reduce the effects of outliers before summing the area.
//******************************************************************************
SonarScanResults ScanForBetterDirection()
{
    static const int SCAN_ANGLE = 90;

    uint16_t leftSum = 0;
    uint16_t leftMax = 0;
    uint16_t rightSum = 0;
    uint16_t rightMax = 0;

    auto ping = PingAt(0); // Move to starting position and do a priming ping for the low-pass filter

    // Scan distances from center to max right angle (right side cw)
    for (auto angle = 1; angle <= SCAN_ANGLE; angle++)
    {
        ping = PingAt(-angle, &ping);
        rightSum += ping;
        rightMax = max(rightMax, ping);
    }

    // Scan distances from max right angle to center (right side ccw)
    for (auto angle = SCAN_ANGLE; angle >= 1; angle--)
    {
        ping = PingAt(-angle, &ping);
        rightSum += ping;
        rightMax = max(rightMax, ping);
    }

    // Scan distances from center to max left (left side ccw)
    for (auto angle = 1; angle <= SCAN_ANGLE; angle++)
    {
        ping = PingAt(angle, &ping);
        leftSum += ping;
        leftMax = max(leftMax, ping);
    }

    // Scan the distance from max left angle to center (left side cw)
    for (auto angle = SCAN_ANGLE; angle >= 1; angle--)
    {
        ping = PingAt(angle, &ping);
        leftSum += ping;
        leftMax = max(leftMax, ping);
    }

    // All done scanning. At this point both sides have been scanned twice.
    // Recenter servo to point straight ahead again
    PanSonar(0);

    auto direction = (leftSum > rightSum) ? 'L' : 'R';

    TRACE(Logger() << F("ScanForBetterDirection, direction=") << direction
                   << F(", leftSum") << leftSum << F(", rightSum=") << rightSum
                   << F(", leftMax=") << leftMax << F(", rightMax=") << rightMax
                   << endl);

    return SonarScanResults { direction, leftSum, rightSum, leftMax, rightMax };
}


//SonarScanResults ScanForBetterDirection2()
//{
//    static const int SCAN_ANGLE = 90;
//
//    auto minAngle = -SCAN_ANGLE;
//    auto maxAngle = SCAN_ANGLE;
//    auto scanData = SonarScanResults(minAngle, maxAngle);
//    auto scanWidth = maxAngle - minAngle;
//    auto cntZones = scanData.CountZones;
//    auto zoneWidth = scanWidth / cntZones;
//    auto centerZone = cntZones / 2;
//    auto zoneCountIsOdd = (cntZones / 2) != 0;
//    auto bestZone = 0;
//
//    PanSonar(minAngle);     // Start sonar at minimum angle
//    delay(450);             // Delay to allow servo to reach start position
//    wdt_reset();
//
//    // Priming ping at start position for low-pass filter
//    float ping = Ping();
//
//    for (auto angle = minAngle; angle <= maxAngle; angle++)
//    {
//        auto index = (angle - minAngle) % zoneWidth;
//
//        PanSonar(angle);
//        delay(5);                       // Delay for servo to move to next position
//        ping = 0.8*ping + 0.2*Ping();   // Use low-pass filter to reduce effect of outliers
//        scanData.ZoneSums[index] += ping;
//        wdt_reset();
//    }
//
//    for (auto i = 0; i < cntZones; i++)
//    {
//        if (scanData.ZoneSums[i] > scanData.ZoneSums[bestZone]) bestZone = i;
//    }
//
//    // The recommended direction angle is the center angle of the best zone
//    scanData.BestAngle = minAngle + (bestZone * zoneWidth) + (zoneWidth / 2);
//
//    if (zoneCountIsOdd && bestZone == centerZone)
//        scanData.BestDirection = 'C';
//    else
//        scanData.BestDirection = (bestZone < centerZone) ? 'L' : 'R';
//
//    PanSonar(0);            // Recenter servo to point straight ahead again
//    delay(450);             // Delay to allow servo to reach start position
//    wdt_reset();
//
//    return scanData;
//}

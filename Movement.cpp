
#define DEBUG 1

#define FULL_SPIN_TIME 3500L

#include "Robot_7_Tank.h"


int currentSpeed = 0;
bool isMoving = false;         // Indicates if the motors are enabled
bool goingSlow = false;


//******************************************************************************
// Movement control methods.
//******************************************************************************

void Go()
{
    TRACE(Logger(F("Go")) << endl);
    SetMotors(currentSpeed, currentSpeed);
}


void Go(int speed)
{
    if (speed != 0)
    {
        TRACE(Logger(F("Go")) << '(' << speed << ')' << endl);
        goingSlow = between(1, speed, SLOW_SPEED);
        currentSpeed = constrain(speed, -MAX_SPEED, MAX_SPEED);
        SetMotors(currentSpeed, currentSpeed);
        isMoving = true;
    }
    else
    {
        Stop();
    }
}


void Stop()
{
    TRACE(Logger(F("Stop")) << endl);
    SetMotors(0, 0);
    isMoving = false;
}


void GoForward()
{
    TRACE(Logger(F("GoForward")) << endl);
    Go(CRUISE_SPEED);
}


void GoSlow()
{
    TRACE(Logger(F("GoSlow")) << endl);
    Go(SLOW_SPEED);
}


void GoBackward()
{
    TRACE(Logger(F("GoBackward")) << endl);
    Go(-CRUISE_SPEED);
}


void GoBackward(uint32_t duration)
{
    TRACE(Logger(F("GoBackward")) << '(' << duration << "ms)" << endl);
    auto timeout = millis() + duration;

    GoBackward();

    while (millis() < timeout) wdt_reset();                 // Keep watchdog timer reset to prevent auto-restart

    Stop();
}


void GoBackward(uint32_t duration, bool(*predicate)())
{
    TRACE(Logger(F("GoBackward")) << '(' << duration << F("ms, predicate)") << endl);
    auto timeout = millis() + duration;

    GoBackward();

    while (millis() < timeout && predicate()) wdt_reset();  // Keep watchdog timer reset to prevent auto-restart

    Stop();
}


void Spin(char direction)
{
    TRACE(Logger(F("Spin")) << '(' << direction << ')' << endl);

    if (direction == 'R')      // Spin to the right
    {
        SetMotors(-CRUISE_SPEED, CRUISE_SPEED);
    }
    else if (direction == 'L') // Spin to the left
    {
        SetMotors(CRUISE_SPEED, -CRUISE_SPEED);
    }
}


void Spin(char direction, uint32_t duration)
{
    TRACE(Logger(F("Spin")) << '(' << direction << ',' << duration << "ms)" << endl);
    auto timeout = millis() + duration;

    Spin(direction);

    while (millis() < timeout) wdt_reset();                 // Keep watchdog timer reset to prevent auto-restart

    Stop();
}


void Spin180(char direction)
{
    TRACE(Logger(F("Spin180")) << '(' << direction << ')' << endl);
    Spin(direction == 'L' ? -180 : 180);
}


//******************************************************************************
/// <summary>
/// Performs a spin of a specific angle, measured in degrees. Negative angles spin
/// right and positive angles spin left. 0 angles do nothing.
/// </summary>
/// <param>angle - The angle to spin, in degrees</param>
/// <remarks>
/// The algorithm uses a simple integration of the gyro rate sensor in the IMU to
/// determine how far we have turned over time.
/// 
/// The gyro reports angular rates about the sensors axes (i.e. the rate of turn
/// in degrees per second) rather than inertial position angles. So, to determine
/// an angle, we have to intergrate the angular rate over time:
/// 
/// angle = sum((w0 + (w1 - w0) / 2.0) * dt) over intervals of dt
/// 
/// This is the trapazoidal integration formula where w0 is the angular rate
/// measurement from the previous iteraton (time t0), w1 is the angular rate
/// measurement at the current iteration (time t1), and dt is the time interval
/// between measurements (t1 - t0). It approximates the area under the angular
/// rate curve over the interval dt as the sum of the area of a rectangle (w0 * dt),
/// representing the contribution from the previous angular rate, and the area
/// of small triangle (w1 -w0) * dt / 2, representing the contribution from the
/// change in angular rate (w1 -w0) over the interval dt. The total angle traversed
/// is the cumulative sum of these small areas from start to finish. To improve
/// accuracy, dt should be very short (on the order of milliseconds).
/// 
/// Since we are only interested in left/right turns of the robot (which are
/// rotations only in the robot's x-y plane) we can just use the z axis angular
/// rate from the gyro (a rotation fixed in the x-y plane is a rotation about
/// the z-axis).This assumes, of course, that the gyro sensor is mounted so that
/// its x-y plane is parallel to and aligned with the robot's x-y plane.
/// 
/// It should be noted that a simple integration of the gyro rates is normally
/// not recommended since the gyro rates drift over time, causing the measured
/// angle to diverge from the true value. However, this caveat is usually made
/// assuming that you are using the gyro to compute attitude angles that must
/// remain stable and accurate over long periods. But we are just measuring how
/// much we have turned so far, and a turn completes in just a few seconds. Over
/// that short time the gyro drift is minimal and can be ignored, as long as you
/// don't need to be super accurate (which we don't in this case).
/// 
/// NOTE: All calculations are in radians!
/// <remarks>
//******************************************************************************
void Spin(int16_t angle)
{
    TRACE(Logger(F("Spin")) << '(' << angle << ')' << endl);

    if (angle == 0) return;
    
    auto spinDirection = (angle < 0) ? 'L' : 'R';

    // If the IMU was not found then approximate the turn angle using
    // a timed spin. Note that all spins are executed at a constant
    // speed (CRUISE_SPEED). The spin time basis, FULL_SPIN_TIME, is
    // the time (in ms) it takes to spin a full 360 degrees at
    // CRUISE_SPEED and needs to be calibrated.
    if (!imu.IsActive())
    {
        Spin(spinDirection, (abs(angle)*FULL_SPIN_TIME) / 360L);
    }
    else
    {
        // Start spin in requested direction
        Spin(spinDirection);

        auto yaw = YawManouver(angle);
        auto startTime = millis();      // Start time for timeout

        // Keep spinning until we complete the yaw manouver (or a 10 second timeout occurs)
        while (!yaw.Update() && (millis() - startTime) < 10000) wdt_reset();

        Stop();
    }
    
    return;




    imu.GetGyro();                              // Initial turn rate

    // Start spin in requested direction
    Spin(spinDirection);

    float targetAngle = abs(angle)*DEG_TO_RAD;  // Target angle
    auto currAngle = 0.0f;                      // How much we have turned so far
    auto t0 = micros();                         // Time of last measurement
    auto w0 = imu.GetGyro().z;                  // Initial gyro reading

    // Use absolute value of current angle since we only need to measure the magnitude 
    // of the turn and not the direction
    for (auto t1 = t0; abs(currAngle) < targetAngle; t1 = micros())
    {
        wdt_reset();

        // Delay at least 4ms between measurements to ensure we have an updated measurement
        // Use UDIFF to compute difference of unsigned numbers (handles 32-bit wrap-around)
        if (UDIFF(t1, t0) < 4000) continue;

        // Get new gyro measurement and compute time since last measurement (in seconds)
        auto w1 = imu.GetGyro().z;
        auto dt = UDIFF(t1, t0) / 1000000.0f;

        // Update turn angle using trapezoidal integration
        currAngle += (w0 + (w1 - w0) / 2.0) * dt;

        TRACE(Logger(F("Spin")) << _FLOAT(dt, 6) << ',' << _FLOAT(w0, 3) << ',' << _FLOAT(w1, 3) << ',' << _FLOAT(currAngle, 3) << endl);

        // Update starting values for next iteration
        w0 = w1;
        t0 = t1;
    }

    Stop();
}


//******************************************************************************
// Set the motor speed
// The speed of both motors is constrained to the range -255 to +255.
//******************************************************************************
void SetMotors(int leftSpeed, int rightSpeed)
{
    //if (!isMoving) leftSpeed = rightSpeed = 0;

    leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

    // If the motors don't rotate in the desired direction you can correct it
    // by changing the sign of the speed in these calls.
    leftMotor.Run(leftSpeed);
    rightMotor.Run(rightSpeed);
}


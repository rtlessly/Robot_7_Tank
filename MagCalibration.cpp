
#define DEBUG 0

#include "Robot_7_Tank.h"


void PerformMagCalibration()
{
    /*--------------------------------------------------------------------------
    This routine performs a real-time calibration of the IMU magnetometer x
    and y axis bias offsets. The basic idea is to take x and y magnetometer
    readings while the robot is spinning in a circle, and note the minimum and
    maximum values on each axis. The bias offset for each axis is the average
    of these minimum and maximum values.

    The magnetometer values can be a bit noisy and are suseptible to producing
    occasional outliers. To ensure that these outliers do not unduly affect
    the calibration, the magnetomer values are smoothed with a low-pass filter
    before updating the min/max values.

    NOTE: This process only calibrates the bias offset for the magnetometer
    x and y axes. This is sufficient for devices that operate primarily in the
    x-y plane. If the mag z-axis value is needed then it must be calibrated
    separately.
    --------------------------------------------------------------------------*/
    TRACE(Logger() << F("MagCalibration.Begin") << endl);

    auto xmin = 1000.0f;
    auto xmax = -1000.0f;
    auto ymin = 1000.0f;
    auto ymax = -1000.0f;
    auto alpha = 0.1f;
    auto beta = 1 - alpha;
    auto mag = imu.GetMagRaw();

    for (auto endtime = millis() + 100; millis() < endtime; mag = imu.GetMagRaw()) delay(10);

    auto x = mag.x;
    auto y = mag.y;

    Spin('R');
    delay(20);

    auto t0 = millis();
    auto endTime = t0 + 6000;

    for (auto t1 = t0; t1 < endTime; t1 = millis())
    {
        wdt_reset();

        if (t1 < (t0 + 10)) continue;

        mag = imu.GetMagRaw();
        x = beta * x + alpha * mag.x;
        y = beta * y + alpha * mag.y;
        xmin = min(x, xmin);
        xmax = max(x, xmax);
        ymin = min(y, ymin);
        ymax = max(y, ymax);

        TRACE(Logger() << F("MagCalibration.Phase1, ") << mag.x << ',' << mag.y << ',' << x << ',' << y << ','
            << _FLOAT(xmin, 2) << ',' << _FLOAT(xmax, 2) << ',' << _FLOAT(ymin, 2) << ',' << _FLOAT(ymax, 2)
            << endl);
    }

    Stop();
    delay(20);
    Spin('L');
    delay(20);

    t0 = millis();
    endTime = t0 + 5000;

    for (auto t1 = t0; t1 < endTime; t1 = millis())
    {
        wdt_reset();

        if (t1 < (t0 + 10)) continue;

        mag = imu.GetMagRaw();
        x = beta * x + alpha * mag.x;
        y = beta * y + alpha * mag.y;
        xmin = min(x, xmin);
        xmax = max(x, xmax);
        ymin = min(y, ymin);
        ymax = max(y, ymax);

        TRACE(Logger() << F("MagCalibration.Phase2, ") << mag.x << ',' << mag.y << ',' << x << ',' << y << ','
            << _FLOAT(xmin, 2) << ',' << _FLOAT(xmax, 2) << ',' << _FLOAT(ymin, 2) << ',' << _FLOAT(ymax, 2)
            << endl);
    }

    Stop();

    auto xBias = (xmin + xmax) / 2.0f;
    auto yBias = (ymin + ymax) / 2.0f;

    imu.SetMagBias(xBias, yBias);

    TRACE(Logger() << F("MagCalibration.Complete, xBias=") << xBias << F(", yBias=") << yBias << endl);
}

ComplimentaryFilterSample
-------------------------

This sample demonstrates a technique called complimentary filtering, whereby
two different measured quantities are used to correct each other.

In this sample, two values are maintained:

  1) A running angular sum computed from gyro readings
  2) An angular value computed from the accelerometer vector

The running gyro value is computed by taking the current angular velocity
in the roll axis.  It takes that and converts it to an amount of rotation
in degrees for this time increment (1/100th of a second).  That incremental
rotation is added to a running total, which is a "gyro only" estimate of the
current orientation.  Over time, numerical imprecision will accumulate, and
temperature drift will also affect it.  You can see this by putting your
finger on the sensor in the middle of the board while it's running.  As the
sensor warms up, the gyro reading will change slightly, and the "zero point"
that was computed on startup will no longer be correct.  This drift will end
up being added to the running total, offsetting the result.  Moving the sensor
very quickly (above its ability to measure) will result in inaccuracy as well.

The accelerometer angle is computed by taking the vector of the accelerometer
X and Z readings, and using the Atan2() function to compute the angle.  This
is a direct computation and therefore not subject to drift.  That said, the
accelerometer is not just measuring gravity.  It measures all forces acting on
the board, so if you shake it, you'll see the readings changing wildly.  The
accelerometer is very sensitive to vibration and noise, so using the measured
value directly is not feasible.


The complimentary filter works by using the measured accelerometer value to
correct the drift in the gyro estimate.  The gyro value is very accurate in
the short term, but over time it will drift away from the correct reading.  On
the other hand, the accelerometer is too noisy to use in the short term, but
over time, they will not drift.  If a small amount of the accelerometer reading
is "mixed in" with every update, it will slowly pull the gyro estimate toward
the correct value.  Since the gyro estimate only drifts off very slowly, this
slow correction is enough to keep it accurate.

This sample allows you to change the amount of gyro and accelerometer mixed
together by moving the slider.  You'll notice that a mix of about 98% gyro
and 2% accelerometer gives good results.  Too much accelerometer and you'll
notice noise, but too little and the gyro might be able to drift off center.

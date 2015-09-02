
Elev8-FC Flight Control firmware
--------------------------------

Modules:

Elev8-Main.spin - Main control loop.  Handles initialization, communication
with configuration app, outer loop of flight control


F32_1_6.spin - 32-bit floating point math routines.  Originally authored by
Jonathan Dummer (Lonesock on the Parallax forums) this is similar to the
Float32 library provided with the Propeller Tool, but generally faster.  It
has been modified to include a "stream processor" whereby a sequence of
operations can be generated into a memory stream.  The code can then run the
operations asynchronously.  Consumes a COG, using nearly all available COG
memory - 1896 of 1984 bytes.  88 bytes remaining in this cog.


FullDuplexSerial-32.spin - A modified version of the standard FullDuplexSerial
module provided with the Propeller Tool.  This version provides 32-byte
transmit and receive buffers, and includes several functions with reduced
state checking for lower overhead, enabling SPIN to send more data without
significantly slowing down the flight control loop.  Consumes a COG, but could
be updated to use one of the multi-serial modules enabling simultaneous I/O
for XBee and serial.


IntPID.spin - Integer PID control object.  This SPIN object is used to run
the PID loops governing the stability controls.  Math is done with integers,
with a user-defined scale factor applied.  At some point I would like to move
the PID functions into F32 streams, but this works well for now.


Propeller Eeprom.spin - I2C communication and EEPROM page read/write module.
This object is used to handle writing variables to EEPROM, allowing the
configuration code to persist user settings for things like gyro drift
compensation.


QuatIMU.spin - Quaternion / Matrix hybrid orientation estimation code.  This
code uses incremental updates to maintain an estimate of the current
orientation in both quaternion and matrix form.  The current quaternion
is rotated by a small-angle quaternion created from the gyro readings.  That
result is converted to a matrix.  The Y axis column of the matrix is compared
against the current accelerometer vector to produce an estimated rotation
error, a portion of which is applied on the next update.  The comparison of
the matrix term and orientation estimate is largely taken from the Discrete
Cosine Matrix mathod described by William Premerlani and Paul Bizard.  This
code relies entirely on the F32 module for computation - it is entirely Spin
and does not take a COG itself.


RC_Receiver.spin - Remote Control Receiever code.  This module converts the
incoming pulse width modulated signals from a standard radio control receiver
into numeric pulse width values, in 1/2 microsecond increments.  Standard R/C
pulse widths are from 1ms to 2ms in length, centered at 1.5ms.  In 1/2uS units,
those values are 2000 (1ms), 4000 (2ms), and 3000 (1.5ms) respectively.
Due to the nature of radio control signals, it is advised that this COG remain
dedicated to this sole task for accuracy.


SBUS-Receiver.spin - Futaba S-BUS Receiver code.  This module decodes the
Futaba S-BUS protocol signals into numeric values.  S-BUS serves the same role
as the PWM signals from a normal radio control receiver, but does so using
a modified serial protocol for accuracy, speed, and robustness.  The output
of the S-BUS module and the RC_Receiver module is interchangeable, but the
S-BUS physically uses only a single wire in addition to power and ground
connections, and provides up to 16 analog channels and 2 binary channels of
input.  Due to the nature of radio control signals, it is advised that this
COG remain dedicated to this sole task for accuracy.


Sensors.spin - Gyro, Accelerometer, Magnetometer, Altimeter, and LED module.
This code reads all the sensors on the Elev8-FC, and writes the LED values to
the WS2812B LEDs.  Since almost all of these devices are high-speed SPI, they
share much of the same interface code, and can be read from very quickly.
This code does the initial setup of the devices, reads their values, and
performs some conditioning of the outputs, like gyro drift compensation.
I also plan to add a median filter for the accelerometer outputs soon.
This COG is primarily PASM, and consumes 1176 of 1984 bytes, with 808 bytes
remaining.  The takes relatively little time to perform its tasks, so more
work can and will be offloaded into this COG.


Servo32-HighRes.spin - ESC/Servo output driver.  This module drives the PWM
outputs for the electronic speed controls and optional servo outputs.  It
supports very high output rates (up to 500Hz) with 1/8uS resolution, or 8000
steps over the standard range of 1ms to 2ms pulse lengths.  This COG only uses
892 of 1984 bytes of code space.  If the desired update rate is below 500Hz,
any "in between" time could be used to run additional functions.



Cogs in use:
1- Elev8-Main / IntPIDs
2- RC Reciever (or) SBus-Receiver
3- Sensors
4- F32_1_6 float math / QuatIMU
5- Servo32-HighRes
6- FullDuplexSerial

Two full cogs currently remain unused.

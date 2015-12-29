This file is part of the ELEV-8 Flight Controller Firmware
for Parallax part #80204, Revision A
Version 1.0
  
Copyright 2015 Parallax Incorporated

ELEV-8 Flight Controller Firmware is free software: you can redistribute it and/or modify it
under the terms of the GNU General Public License as published by the Free Software Foundation, 
either version 3 of the License, or (at your option) any later version.

ELEV-8 Flight Controller Firmware is distributed in the hope that it will be useful, but 
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with the ELEV-8 Flight Controller Firmware.  If not, see <http://www.gnu.org/licenses/>.
  
Written by Jason Dorie

--------------------------------

Current Flight Mode Capabilities:

Stable:
  - Automatic leveling
  - Tilt-compensated thrust
  - Accelerometer disturbance thrust adjustment

Manual:
  - Gyro stabiization - basically a heading hold gyro on all three axis

--------------------------------

Modules:

Elev8-Main - Main control loop.  Handles initialization, communication
with configuration app, outer loop of flight control


Battery - This module contains functions for computing the current battery
voltage, based on resistor/capacitor charge time.  Counter B is used to
measure the charge time.


Beep - This module contains functions to sound the piezo buzzer, either
by directly toggling the pin, or through the use of Counter A, allowing
the buzzer to be sounded while performing other tasks.


CommLink - This module is responsible for creating the data packets sent
to the GroundStation software.  Packets have a standard header, and a
checksum.  Functions are included for sending a complete packet in one
call, or assembling a packet over multiple function calls.


Eeprom - I2C communication and EEPROM page read/write module.
This object is used to handle writing variables to EEPROM, allowing the
configuration code to persist user settings for things like gyro drift
and accelerometer offset compensation.  Ported from Spin to C/C++.


F32 - 32-bit floating point math routines.  Originally authored by
Jonathan Dummer (Lonesock on the Parallax forums) this is similar to the
Float32 library provided with the Propeller Tool, but generally faster.  It
has been modified to include a "stream processor" whereby a sequence of
operations can be generated into a memory stream.  The code can then run the
operations asynchronously.  Consumes a COG, using nearly all available COG
memory.


IntPID - Integer PID control object.  This object is used to run the PID loops
governing the stability controls.  Math is done with integers,
with a user-defined scale factor applied.  At some point I would like to move
the PID functions into F32 streams, but this works well for now.


Pins - This file contains the constant definitions for which devices are
connected to which physical pins on the Propeller.


Prefs - User preferences storage.  This module handles the storage of user
preferences to the EEPROM, setting defaults, and ensuring integrity of the
data with checksums.


QuatIMU - Quaternion / Matrix hybrid orientation estimation code.  This
code uses incremental updates to maintain an estimate of the current
orientation in both quaternion and matrix form.  The current quaternion
is rotated by a small-angle quaternion created from the gyro readings.  That
result is converted to a matrix.  The Y axis column of the matrix is compared
against the current accelerometer vector to produce an estimated rotation
error, a portion of which is applied on the next update.  The comparison of
the matrix term and orientation estimate is largely taken from the Discrete
Cosine Matrix mathod described by William Premerlani and Paul Bizard.  This
code relies entirely on the F32 module for computation - it is almost entirely
data structures which are the instruction streams for the F32 stream processor,
and therefore does not take a COG itself.

QuatIMU does altitude estimation by fusing accelerometer and altimeter readings.
Gravity is subtracted from the current accelerometer vector, the vector is
rotated into world-space, and a velocity estimate is updated with it.  Another
velocity estimate is computed from the rate of change of the pressure-altitude
reading.  The accelerometer-based value is filtered using the altimeter value,
and the result is integrated into an altitude estimate.  The altitude estimate
is then itself filtered using the pressure-altitude value.

QuatIMU additionally includes functions to update the target orientation
quaternion, either in manual mode (incremental updates) or stable mode (absolute
orientation based on stick position).  The desired target quaternion is then
compared against the current orientation quaternion and a rotational difference
is computed.  This difference is what is fed into the PID controllers for the
three flight axis.


RC - Remote Control Receiever code.  This module converts the incoming pulse width
modulated signals from a standard radio control receiver into numeric pulse width
values, in 1/2 microsecond increments.  Standard R/C pulse widths are from 1ms to
2ms in length, centered at 1.5ms.  In 1/2uS units, those values are 2000 (1ms),
4000 (2ms), and 3000 (1.5ms) respectively. Due to the nature of radio control
signals, it is advised that this COG remain dedicated to this sole task for
accuracy.  This module has two PASM drivers it can use, one that monitors 8 pins
for 8 independent PWM inputs, and one that monitors a single input for a PPM stream.
The output is identical between both drivers.


SBUS-Receiver - Futaba S-BUS Receiver code.  This module decodes the
Futaba S-BUS protocol signals into numeric values.  S-BUS serves the same role
as the PWM signals from a normal radio control receiver, but does so using
a modified serial protocol for accuracy, speed, and robustness.  The output
of the S-BUS module and the RC_Receiver module is interchangeable, but the
S-BUS physically uses only a single wire in addition to power and ground
connections, and provides up to 16 analog channels and 2 binary channels of
input.  Due to the nature of radio control signals, it is advised that this
COG remain dedicated to this sole task for accuracy.


Sensors - Gyro, Accelerometer, Magnetometer, Altimeter, and LED module.
This code reads all the sensors on the Elev8-FC, and writes the LED values to
the WS2812B LEDs.  Since almost all of these devices are high-speed SPI, they
share much of the same interface code, and can be read from very quickly.
This code does the initial setup of the devices, reads their values, and
performs some conditioning of the outputs, like gyro drift compensation,
median filtering of the accelerometer outputs, and conversion of the
barometric pressure reading to an altitude estimate, done using a lookup table.


Serial_4x_driver - Ported from Spin, this driver from Tracey Allen runs
four simultaneous full-duplex serial ports at different baud rates in a single
cog.  This allows the Elev8FC to communicate over USB, XBee, and two additional
serial simultaneously.


Servo32-HighRes - ESC/Servo output driver.  This module drives the PWM
outputs for the electronic speed controls and optional servo outputs.  It
supports very high output rates (up to 500Hz) with 1/8uS resolution, or 8000
steps over the standard range of 1ms to 2ms pulse lengths.  This COG only uses
892 of 1984 bytes of code space.  If the desired update rate is below 500Hz,
any "in between" time could be used to run additional functions.

--------------------------------

Cogs in use:
1- Elev8-Main / IntPIDs
2- RC Reciever (or) SBus-Receiver
3- Sensors
4- F32 float math / QuatIMU
5- Servo32-HighRes
6- Serial_4X

Two full cogs currently remain unused.

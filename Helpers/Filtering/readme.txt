FilterSample
------------

This sample demonstrates the use of signal filtering based on the common R/C
filter used in electronics.  An R/C filter, or Resistor/Capacitor filter,
uses a resistor and a capacitor to reduce noise from the incoming signal.
The values of the two components form a ratio, and that ratio controls the
amount of filtering performed.  In the digital version, the ratio itself
drives an equation:

   Value = Value + (NewValue - Value) * Ratio

The difference between the current value and the new value is multiplied
by the ratio value (which is between 0.0 and 1.0).  That result is added
to the current value.  In plain language, the filter allows the current
value to "move toward" the new value by some fraction.  If the "ratio"
value is 1.0, no filtering happens - the existing value takes a complete
step toward the new value, moving by the full amount of their difference.
However, if the "ratio" is less than 1.0, say 0.5, then each new sample
only influences the current value by 50 percent - The current value moves
half the distance between itself and the new value at each update.

A value of zero prevents the current value from moving at all, so the
filter ratio value must be greater than zero, and less than or equal
to one.  Lower values produce a smoother filtered result, but because
they prevent the value from changing rapidly, they also introduce more
lag.  Any filter must balance noise and lag.


This sample shows the current Gyro Y value from the Elev8 flight
controller in a graph.  The slider along the bottom allows you to change
the strength of the filter being applied.  The red line shows the original
unfiltered value, and the blue line shows the output of the filter.

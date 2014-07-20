TimerSerial
===========

Timer based software serial for PIC32

This library emulates a single serial port (full duplex) on a pair of generic IO pins.  It uses
a timer to operate (timer 1), so also requires my Timer library (https://github.com/majenkotech/Timer)
to operate.

The operation is completely "in the background" so it acts just like a standard hardware
serial port with the exact same (Stream based) interface.

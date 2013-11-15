am2301
======

Kernel module for AM2301 temperature/humidity module

This is a driver for Raspberry PI.
It uses pin 24 as default pin, configurable as module parameter.

The driver uses by default a procfs interface, reporting T, RH and read quality.

pi@pi ~ $ cat /proc/am2301
T     :         18.6
RH    :         76.1
QUAL  :         263/351 74%

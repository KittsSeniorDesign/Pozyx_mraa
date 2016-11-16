# Pozyx_mraa
The Arduino library for use with the pozyx shield, but it also works on the Intel Edison because of the Wire.h provided here. **Notice it is not the same Wire.h that Ardunio uses, and requires mraa to be installed**

The library requires **firmware version 1.0** installed on the Pozyx devices.

Documentation for the library can be found here:
https://www.pozyx.io/Documentation/Datasheet/arduino

The following folders can be found together with this library:

1.  **examples**. These example scripts showcase some basic functionality of the Pozyx device, each example comes with a tutorial that can be found on the pozyx website https://www.pozyx.io/Documentation
2.  **unit_test**. This folder contains a collection of Arduino scripts that can be run to test certain functionalities of the Pozyx device. They also serve as some good examples for some Arduino library functions.
3.  **useful**. This folder contains a number of useful Arduino sketches that provide basic functionality such as discovering all pozyx devices or configuring them.

# xv11lidar
XV11 LIDAR C/C++ Communication Library

## Repository Intention

Intention is for use as submodule in other projects.

## Notes

This library doesn't control the motor, only the LIDAR communication.

It is low level library that lets you read as little data as you need.
This is important if you want to have super-precise timestamping scheme.

This power comes at a price - the library returns LIDAR frames in raw form output by the device.

The library has only 3 functions:
- InitLaser
- ReadLaser (you will probably want to call this in a loop)
- CloseLaser

## Implementations

You can use those repositories as examples.

### [ev3laser] module

[ev3laser] in [ev3dev-mapping-modules] repository uses [ev3dev] C++ bindings to spin the motor and xv11lidar library to continuously 
read from the LIDAR, timestamp the data and send over UDP to [ev3dev-mapping-ui].

### [xv11lidar-test]

[xv11lidar-test] is a small C utility for testing the LIDAR. Currently it uses outdated version of the xv11lidar library.
Utility can read from the the LIDAR continuously or predetermined amount of data. It outputs either csv data or raw-binary data to standard output.
The output can be read on console, redirected to file or redirected to other program as needed.
The motor has to be controlled outside of the utility and is controlled from the bash in the repository documentation.

[ev3laser]: https://github.com/bmegli/ev3dev-mapping-modules/tree/master/ev3laser
[ev3dev-mapping-modules]: https://github.com/bmegli/ev3dev-mapping-modules
[ev3dev-mapping-ui]: https://github.com/bmegli/ev3dev-mapping-ui
[ev3dev]: http://www.ev3dev.org/
[xv11lidar-test]: https://github.com/bmegli/xv11lidar-test
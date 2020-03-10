# xv11lidar
XV11 LIDAR C/C++ Communication Library

# Deprecated

As of 2020 this library is deprecated. It is no longer maintained in any way.

It was implented with the false assumption.\
I assumed that once the proper communication is established it will work.\
Turns out the harsh real world is different.

Despite this flaw the XV11 LIDAR with this library served me well for years in many projects.

[![How to interface XV11 LIDAR to EV3 using ev3dev](http://img.youtube.com/vi/G6uVg34VzHw/0.jpg)](https://youtu.be/G6uVg34VzHw)

[![EV3 Gyro vs CruizCore XG1300L vs Odometry - Position Estimation](http://img.youtube.com/vi/vzND_ISdhEs/0.jpg)](https://www.youtube.com/watch?v=vzND_ISdhEs)

[![3D mapping/scanning project with ev3dev OS and Unity UI](http://img.youtube.com/vi/9o_Fi8bHdvs/0.jpg)](https://youtu.be/9o_Fi8bHdvs)

Rest in peace XV11 LIDAR. Rest in peace xv11lidar.

In 2020 (finally!) there are many better devices to use in projects:
- a range of low cost off the shelf lidars
- a family of Realsense devices
- Azure Kinect
- ZED cameras
- and plenty of other I never had the chance to play with


## Repository Intention

Intention is for use as submodule in other projects.

## Notes

This library doesn't control the motor, only the LIDAR communication.

It is low level library that lets you read as little data as you need.
This is important if you want to have super-precise timestamping scheme.

The library returns LIDAR frames in raw form output by the device.

The library has only 3 functions:
- xv11lidar_init
- xv11lidar_read (you will probably want to call this in a loop)
- xv11lidar_close

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

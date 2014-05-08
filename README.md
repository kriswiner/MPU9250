MPU-9250
========

Arduino sketch for MPU-9250 9DoF sensor with AHRS sensor fusion

Demonstrate MPU-9250 basic functionality including parameterizing the register addresses, initializing the sensor, 
getting properly scaled accelerometer, gyroscope, and magnetometer data out, calibration of sensors.
Added display functions to allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.

MPU-9250
========

Arduino sketch for MPU-9250 9 DoF sensor with AHRS sensor fusion

Demonstrate MPU-9250 basic functionality including parameterizing the register addresses, initializing the sensor, 
getting properly scaled accelerometer, gyroscope, and magnetometer data out, calibration and self-test of sensors.
Added display functions to allow display to on-breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.

A discussion of the use and limitations of this sensor and sensor fusion in general is found ![here.]
(https://github.com/kriswiner/MPU-6050/wiki/Affordable-9-DoF-Sensor-Fusion)

I have also added a program to allow sensor fusion using the MPU-9250 9-axis motion sensor with the STM32F401 Nucleo board using the mbed compiler. The STM32F401 achieves a sensor fusion filter update rate using the Madgwick MARG fusion filter of 4800 Hz running the M4 Cortex ARM processor at 84 MHz; compare to the sensor fusion update rate of 2120 Hz achieved using the same filter with the Teensy 3.1 running its M4 Cortex ARM processor at 96 MHz.

One reason for this difference is the single-precision floating point engine embedded in the STM32F401 core. While both ARM processors achieve impressive rates of filtering, really more than necessary for most applications, the factor of two difference translates into much lower power consumption for the same sensor fusion performance. If adequate sensor fusion filtering, say, 1000 Hz, can be achieved at much lower processor clock speed, then over all power consumption will be reduced. This really matters for wearable and other portable motion sensing and control applications.

I added a version of the basic sketch that uses the i2c_t3.h 'Wire' library specifically designed for Teensy 3.1. It allows easy access to Teensy-specific  capabilities such as specification of which set of hardware i2c pins will be used, the bus speed (up to 1 MHz!) and also allows master and/or slave designation to handle multiplexing between i2c devices. See www.pjrc.com/teensy and ![here](http://forum.pjrc.com/threads/21680-New-I2C-library-for-Teensy3) for details.

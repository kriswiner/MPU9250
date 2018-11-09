MPU-9250 9 DOF IMU Arduino Library
==================================

[![MPU-9250 Breakout](https://cdn.sparkfun.com/assets/parts/1/1/3/0/6/13762-00a.jpg)](https://cdn.sparkfun.com/assets/parts/1/1/3/0/6/13762-00a.jpg)

[*MPU-9250 Breakout (SEN-13762)*](https://www.sparkfun.com/products/13762)

This is an arduino IDE library to control and read from the MPU-9250. It provides access to accelerometer, gyroscope, and magnetometer sensor readings.  It should be compatible with any Arduino-compatible development board. It has been thoroughly tested with Arduino Pro Mini.

If you're looking for a more advanced library, which takes advantage of the MPU-9250's digital motion processing (DMP) features (tap-detection, pedometer, orientation, quaternion-calculation), and if you have a more advanced Arduino development board (e.g. [Arduino Zero](https://store.arduino.cc/usa/arduino-zero), [SparkFun SAMD21 Breakout](https://www.sparkfun.com/products/13664)), check out the [SparkFun MPU-9250 DMP Library](https://github.com/sparkfun/SparkFun_MPU-9250-DMP_Arduino_Library).

Repository Contents
-------------------

* **/examples** &mdash; Example sketch for the library (.ino). Run this from the Arduino IDE.
* **/src** &mdash; Source files for the library (.cpp, .h).
* **keywords.txt** &mdash; Keywords from this library that will be highlighted in the Arduino IDE.
* **library.properties** &mdash; General library properties for the Arduino package manager.

Example Briefs
--------------

* MPU9250BasicAHRS &mdash; Prints out sensor data with some sane default configuration parameters

Documentation
--------------

* **[Installing an Arduino Library Guide](https://learn.sparkfun.com/tutorials/installing-an-arduino-library)** &mdash; Basic information on how to install an Arduino library.
* **[Product Repository](https://github.com/sparkfun/MPU-9250_Breakout)** &mdash; Main repository (including hardware files) for the MPU-9250 Breakout.
* **[Datasheet](https://cdn.sparkfun.com/assets/learn_tutorials/5/5/0/MPU9250REV1.0.pdf)** &mdash; Datasheet containing part of the product documentation.
* **[Register map](https://cdn.sparkfun.com/assets/learn_tutorials/5/5/0/MPU-9250-Register-Map.pdf)** &mdash; Register map containing the rest of the product documentation.
* **[Hookup Guide](https://learn.sparkfun.com/tutorials/MPU-9250-hookup-guide)** &mdash; Basic hookup guide for the MPU-9250 Breakout.

Products that use this Library
---------------------------------

* [SEN-13762](https://www.sparkfun.com/products/13762) &mdash; MPU-9250 Breakout board

Version History
---------------

* [V 1.0.0](https://github.com/sparkfun/SparkFun_MPU-9250_6_DOF_IMU_Breakout_Arduino_Library/releases/tag/V_1.0.0) &mdash; Initial commit of Arduino 1.5+ compatible library.

License Information
-------------------

This product is _**open source**_!

Code is a lightly modified version of Kris Winer's [code](https://github.com/kriswiner/MPU-9250) which was licensed as Beerware. It's unclear the license of the code in quaternionFilters.ino.

If you have any questions or concerns on licensing, please contact techsupport@sparkfun.com.

Distributed as-is; no warranty is given.

- Your friends at SparkFun.

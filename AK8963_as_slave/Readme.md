Simple sketch to illustrate the method of treating the AK8963C embedded in the MPU9250 as a slave on the MPU9250's internal master I2C bus. 
This sketch uses I2C to communicate with the host MCU although a simple change will allow SPI to be used.

The idea here is that the AK8963C, which always has I2C address 0x0C, can be managed by the MPU9250 it is embedded in so that multiple MPU9250's can be on the same host I2C bus and the magnetometers can be kept straight.

There is some work required to convert this sketch into a sketch that can manage two MPU9250s ion the same I2C bus but with this method it should be straightforward to do so without collisions between the AK8963C.

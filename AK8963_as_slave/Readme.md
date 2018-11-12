Simple sketch to illustrate the method of treating the AK8963C embedded in the MPU9250 as a slave on the MPU9250's internal master I2C bus. 
This sketch uses I2C to communicate between the host MCU and MPU9250 on 0x68 or 0x69, although a simple change will allow SPI to be used.

The idea here is that the AK8963C, which always has I2C address 0x0C, can be managed by the MPU9250 it is embedded in so that multiple MPU9250's can be on the same host I2C bus and the magnetometers can be kept straight. The host only reads and write to MPU9250 registers, never talks with the AK8963C directly and, in effect, the AK8963C becomes part of the integrated MPU9250 and all transactions can be performed by reference to the 0x68 or 0x69 I2C address of the "host" MPU9250. This avoids the conflict that usually occurs when two AK8963C are exposed directly to the MCU host on the master I2C bus since both AK8963C will have the same I2C address.  

There is some work required yet to convert this into a sketch that can manage two MPU9250s on the same I2C bus but with this method it should be straightforward (I have done it and it works well) to do so without collisions between the AK8963C.

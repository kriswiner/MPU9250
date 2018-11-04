Arduino sketch that treats two MPU9250, one at address 0x68 and one at 0x69 on the same I2C bus, as slaves to the Ladybug MCU host
but local masters to their AK8963C magnetometers. This removes any ambiguity concerning which mag data comes from which MPU9250 since 
the host only communicates with the MPU9250 never the AK8963C. The sketch allows basic calibration of all sensors, retreives the sensor 
data at an appropriate (200 Hz for accel/gyro, 100 Hz for mag) rate, and calculates quaternions from the data using the Madgwick iterative 
fusion method for each set of data separately, and then estimates the yaw/pitch/roll, gravity/linear acceleration for each of the two sensors.

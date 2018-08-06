#include "quaternionFilters.h"
#include "MPU9250.h"

#define I2Cclock 400000
#define I2Cport Wire
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0   // Use either this line or the next to select which I2C address your device is using
//#define MPU9250_ADDRESS MPU9250_ADDRESS_AD1

MPU9250 myIMU0(MPU9250_ADDRESS_AD0, I2Cport, I2Cclock);
MPU9250 myIMU1(MPU9250_ADDRESS_AD1, I2Cport, I2Cclock);

 byte c = 0x00;
 byte d = 0x00;
 bool ledOn = true;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);
  while(!Serial){};

  pinMode(13, OUTPUT);
  
}

void loop() {
  // put your main code here, to run repeatedly:

  c = myIMU0.readByte(MPU9250_ADDRESS_AD0, WHO_AM_I_MPU9250);
  d = myIMU1.readByte(MPU9250_ADDRESS_AD1, WHO_AM_I_MPU9250);

  Serial.print("Received AD0: 0x");
  Serial.print(c, HEX);
  Serial.print(", AD1: 0x");
  Serial.println(d, HEX);
  digitalWrite(13, ledOn);
  ledOn = !ledOn;
  delay(100);

}

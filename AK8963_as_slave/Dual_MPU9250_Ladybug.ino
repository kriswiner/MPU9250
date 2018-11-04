/* 07/6/2017 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer
 * 
 Demonstrate basic MPU-9250 functionality including parameterizing the register addresses, initializing the sensor, 
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. 
 Addition of 9 DoF sensor fusion using open source Madgwick and Mahony filter algorithms. 
 Sketch runs on the 3.3 V Dragonfly STM32L476 Breakout Board.

 The BME280 is a simple but high resolution pressure/humidity/temperature sensor, which can be used in its high resolution
 mode but with power consumption of 20 microAmp, or in a lower resolution mode with power consumption of
 only 1 microAmp. The choice will depend on the application.
 
 SDA and SCL have 4K7 pull-up resistors (to 3.3V).

 Library may be used freely and without limit with attribution.
*/
 
#include "Wire.h"   
#include "MPU9250.h"
#include "RTC.h"

#define SerialDebug true   // set to true to get Serial output for debugging


// RTC set time using STM32L4 natve RTC class
/* Change these values to set the current initial time */
const uint8_t seconds = 0;
const uint8_t minutes = 15;
const uint8_t hours = 17;

/* Change these values to set the current initial date */
const uint8_t day = 4;
const uint8_t month = 7;
const uint8_t year = 17;

uint8_t Seconds, Minutes, Hours, Day, Month, Year;

bool alarmFlag = false; // for RTC alarm interrupt


// MPU9250 Configuration
// Specify sensor full scale
/* Choices are:
 *  Gscale: GFS_250 == 250 dps, GFS_500 DPS == 500 dps, GFS_1000 == 1000 dps, and GFS_2000DPS == 2000 degrees per second gyro full scale
 *  Ascale: AFS_2G == 2 g, AFS_4G == 4 g, AFS_8G == 8 g, and AFS_16G == 16 g accelerometer full scale
 *  Mscale: MFS_14BITS == 0.6 mG per LSB and MFS_16BITS == 0.15 mG per LSB
 *  Mmode: Mmode == M_8Hz for 8 Hz data rate or Mmode = M_100Hz for 100 Hz data rate
 *  (1 + sampleRate) is a simple divisor of the fundamental 1000 kHz rate of the gyro and accel, so 
 *  sampleRate = 0x00 means 1 kHz sample rate for both accel and gyro, 0x04 means 200 Hz, etc.
 */
uint8_t Gscale = GFS_250DPS, Ascale = AFS_2G, Mscale = MFS_16BITS, Mmode = M_100Hz, sampleRate = 0x04;         
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors
float motion = 0; // check on linear acceleration to determine motion
// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float pi = 3.141592653589793238462643383279502884f;
float GyroMeasError = pi * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = pi * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float beta = sqrtf(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrtf(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
bool wakeup;

// Pin definitions
int  intPin = 9;  //  MPU9250 interrupt
int  myLed  = 13; // red led

bool intFlag = false;
bool newMagData = false;

int16_t MPU9250Data[7]; // used to read all 14 bytes at once from the MPU9250 accel/gyro
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float   magCalibration[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float   temperature;    // Stores the MPU9250 internal chip temperature in degrees Celsius
float   SelfTest[6];    // holds results of gyro and accelerometer self test

// These can be measured once and entered here or can be calculated each time the device is powered on
float   gyroBias[3] = {0.96, -0.21, 0.12}, accelBias[3] = {0.00299, -0.00916, 0.00952};
float   magBias[3] = {71.04, 122.43, -36.90}, magScale[3]  = {1.01, 1.03, 0.96}; // Bias corrections for gyro and accelerometer


uint32_t delt_t = 0;                      // used to control display output rate
uint32_t count = 0, sumCount = 0;         // used to control display output rate
float pitch, yaw, roll;                   // absolute orientation
float a12, a22, a31, a32, a33;            // rotation matrix coefficients for Euler angles and gravity components
float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;                         // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float lin_ax, lin_ay, lin_az;             // linear acceleration (acceleration with gravity component subtracted)
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

MPU9250 MPU9250(intPin); // instantiate MPU9250 class


void setup()
{
  Serial.begin(115200);
  delay(1000);
  
  Wire.begin(); // set master mode, default on SDA/SCL for Ladybug   
  Wire.setClock(400000); // I2C frequency at 400 kHz
  delay(1000);

  MPU9250.I2Cscan(); // should detect BME280 at 0x77, MPU9250 at 0x71 
  
  // Set up the interrupt pin, it's set as active high, push-pull
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH); // start with orange led on (active HIGH)

  pinMode(intPin, INPUT);

   /* Configure the MPU9250 */
  // Read the WHO_AM_I register, this is a good test of communication
  Serial.println("MPU9250 9-axis motion sensor...");
  uint8_t c = MPU9250.getMPU9250ID();
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);
  delay(1000);
  
  if (c == 0x71 ) // WHO_AM_I should always be 0x71 for MPU9250, 0x73 for MPU9255 
  {  
    Serial.println("MPU9250 is online...");
    
    MPU9250.resetMPU9250(); // start by resetting MPU9250
    
    MPU9250.SelfTest(SelfTest); // Start by performing self test and reporting values
    Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5],1); Serial.println("% of factory value");
    delay(1000);

  // get sensor resolutions, only need to do this once
  aRes = MPU9250.getAres(Ascale);
  gRes = MPU9250.getGres(Gscale);
  mRes = MPU9250.getMres(Mscale);

 // Comment out if using pre-measured, pre-stored offset biases
  MPU9250.calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
  Serial.println("accel biases (mg)"); Serial.println(1000.*accelBias[0]); Serial.println(1000.*accelBias[1]); Serial.println(1000.*accelBias[2]);
  Serial.println("gyro biases (dps)"); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);
  delay(1000); 
  
  MPU9250.initMPU9250(Ascale, Gscale, sampleRate); 
  Serial.println("MPU9250 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
  
  // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
  byte d = MPU9250.getAK8963CID();  // Read WHO_AM_I register for AK8963
  Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);
  delay(1000); 
  
  // Get magnetometer calibration from AK8963 ROM
  MPU9250.initAK8963Slave(Mscale, Mmode, magCalibration); Serial.println("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer

 // Comment out if using pre-measured, pre-stored offset biases
//  MPU9250.magcalMPU9250(magBias, magScale);
  Serial.println("AK8963 mag biases (mG)"); Serial.println(magBias[0]); Serial.println(magBias[1]); Serial.println(magBias[2]); 
  Serial.println("AK8963 mag scale (mG)"); Serial.println(magScale[0]); Serial.println(magScale[1]); Serial.println(magScale[2]); 
  delay(2000); // add delay to see results before serial spew of data

  if(SerialDebug) {
  Serial.println("Calibration values: ");
  Serial.print("X-Axis sensitivity adjustment value "); Serial.println(magCalibration[0], 2);
  Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(magCalibration[1], 2);
  Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(magCalibration[2], 2);

  attachInterrupt(intPin, myinthandler, RISING);  // define interrupt for intPin output of MPU9250
  }
  
  }
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }


   digitalWrite(myLed, LOW); // turn off led when using flash memory

  // Set the time
  RTC.setHours(hours);
  RTC.setMinutes(minutes);
  RTC.setSeconds(seconds);

  // Set the date
  RTC.setDay(day);
  RTC.setMonth(month);
  RTC.setYear(year);
  
  /* Set up the RTC alarm interrupt */
  RTC.enableAlarm(RTC.MATCH_ANY); // alarm once a second
  
  RTC.attachInterrupt(alarmMatch); // interrupt every time the alarm sounds
  
}

void loop()
{  
   // If intPin goes high, either all data registers have new data
   if(intFlag == true) {   // On interrupt, read data
      intFlag = false;     // reset newData flag
      
     MPU9250.readMPU9250Data(MPU9250Data); // INT cleared on any read
   
    // Now we'll calculate the accleration value into actual g's
     ax = (float)MPU9250Data[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
     ay = (float)MPU9250Data[1]*aRes - accelBias[1];   
     az = (float)MPU9250Data[2]*aRes - accelBias[2];  

    // Calculate the gyro value into actual degrees per second
     gx = (float)MPU9250Data[4]*gRes;  // get actual gyro value, this depends on scale being set
     gy = (float)MPU9250Data[5]*gRes;  
     gz = (float)MPU9250Data[6]*gRes; 
  
//    if( MPU9250.checkNewMagData() == true) { // wait for magnetometer data ready bit to be set
      MPU9250.readMagData(magCount);  // Read the x/y/z adc values
  
    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
      mx = (float)magCount[0]*mRes*magCalibration[0] - magBias[0];  // get actual magnetometer value, this depends on scale being set
      my = (float)magCount[1]*mRes*magCalibration[1] - magBias[1];  
      mz = (float)magCount[2]*mRes*magCalibration[2] - magBias[2];  
      mx *= magScale[0];
      my *= magScale[1];
      mz *= magScale[2]; 
//    }
   
  
    for(uint8_t i = 0; i < 10; i++) { // iterate a fixed number of times per data read cycle
    Now = micros();
    deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate = Now;

    sum += deltat; // sum for averaging filter update rate
    sumCount++;

    MadgwickQuaternionUpdate(-ax, +ay, +az, gx*pi/180.0f, -gy*pi/180.0f, -gz*pi/180.0f,  my,  -mx, mz);
    }

    /* end of MPU9250 interrupt handling */
   }
  

      
    if(alarmFlag) { // update RTC output (serial display) whenever the RTC alarm condition is achieved  
       alarmFlag = false;

     // Read RTC
    Serial.println("RTC:");
    Day = RTC.getDay();
    Month = RTC.getMonth();
    Year = RTC.getYear();
    Seconds = RTC.getSeconds();
    Minutes = RTC.getMinutes();
    Hours   = RTC.getHours();     
    if(Hours < 10) {Serial.print("0"); Serial.print(Hours);} else Serial.print(Hours);
    Serial.print(":"); 
    if(Minutes < 10) {Serial.print("0"); Serial.print(Minutes);} else Serial.print(Minutes); 
    Serial.print(":"); 
    if(Seconds < 10) {Serial.print("0"); Serial.println(Seconds);} else Serial.println(Seconds);  

    Serial.print(Month); Serial.print("/"); Serial.print(Day); Serial.print("/"); Serial.println(Year);
    Serial.println(" ");
    
    if(SerialDebug) {
    Serial.print("ax = "); Serial.print((int)1000*ax);  
    Serial.print(" ay = "); Serial.print((int)1000*ay); 
    Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
    Serial.print("gx = "); Serial.print( gx, 2); 
    Serial.print(" gy = "); Serial.print( gy, 2); 
    Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");
    Serial.print("mx = "); Serial.print( (int)mx ); 
    Serial.print(" my = "); Serial.print( (int)my ); 
    Serial.print(" mz = "); Serial.print( (int)mz ); Serial.println(" mG");
    
    Serial.print("q0 = "); Serial.print(q[0]);
    Serial.print(" qx = "); Serial.print(q[1]); 
    Serial.print(" qy = "); Serial.print(q[2]); 
    Serial.print(" qz = "); Serial.println(q[3]); 

    temperature = ((float) MPU9250Data[3]) / 333.87f + 21.0f; // Gyro chip temperature in degrees Centigrade
    // Print temperature in degrees Centigrade      
    Serial.print("Gyro temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
    }               
   
    a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
    a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
    a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
    a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
    a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
    pitch = -asinf(a32);
    roll  = atan2f(a31, a33);
    yaw   = atan2f(a12, a22);
    pitch *= 180.0f / pi;
    yaw   *= 180.0f / pi; 
    yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    if(yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360
    roll  *= 180.0f / pi;
    lin_ax = ax + a31;
    lin_ay = ay + a32;
    lin_az = az - a33;

    if(SerialDebug) {
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(yaw, 2);
    Serial.print(", ");
    Serial.print(pitch, 2);
    Serial.print(", ");
    Serial.println(roll, 2);

    Serial.print("Grav_x, Grav_y, Grav_z: ");
    Serial.print(-a31*1000.0f, 2);
    Serial.print(", ");
    Serial.print(-a32*1000.0f, 2);
    Serial.print(", ");
    Serial.print(a33*1000.0f, 2);  Serial.println(" mg");
    Serial.print("Lin_ax, Lin_ay, Lin_az: ");
    Serial.print(lin_ax*1000.0f, 2);
    Serial.print(", ");
    Serial.print(lin_ay*1000.0f, 2);
    Serial.print(", ");
    Serial.print(lin_az*1000.0f, 2);  Serial.println(" mg");
    
    Serial.print("rate = "); Serial.print((float)sumCount/sum, 2); Serial.println(" Hz");
    sumCount = 0;
    sum = 0;    
    }

} /* end of alarm handling */



}

//===================================================================================================================
//====== Set of useful functions
//===================================================================================================================

void myinthandler()
{
  intFlag = true;
}

void alarmMatch()
{
  alarmFlag = true;
}

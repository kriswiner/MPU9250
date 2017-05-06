/* MPU9250_MS5637_t3 Basic Example Code
 by: Kris Winer
 date: April 1, 2014
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 Demonstrate basic MPU-9250 functionality including parameterizing the register addresses, initializing the sensor, 
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. Added display functions to 
 allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and 
 Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.
 
 This sketch is intended specifically for the MPU9250+BMP280 Add-on shield for the Teensy 3.1.
 It uses SDA/SCL on pins 17/16, respectively, and it uses the Teensy 3.1-specific Wire library i2c_t3.h.
 The MS5637 is a simple but high resolution pressure sensor, which can be used in its high resolution
 mode but with power consumption of 20 microAmp, or in a lower resolution mode with power consumption of
 only 1 microAmp. The choice will depend on the application.
 
 SDA and SCL should have external pull-up resistors (to 3.3V).
 4K7 resistors are on the MPU9250+MS5637 breakout board.
 
 Hardware setup:
 MPU9250 Breakout --------- Arduino
 VDD ---------------------- 3.3V
 VDDI --------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND
 
 Note: The MPU9250 is an I2C sensor and uses the Arduino Wire library. 
 Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
 We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
 */
//#include "Wire.h"   
#include <i2c_t3.h>
#include <SPI.h>

// See also MPU-9250 Register Map and Descriptions, Revision 4.0, RM-MPU-9250A-00, Rev. 1.4, 9/9/2013 for registers not listed in 
// above document; the MPU9250 and MPU9150 are virtually identical but the latter has a different register map
//
//Magnetometer Registers
#define AK8963_ADDRESS   0x0C
#define WHO_AM_I_AK8963  0x00 // should return 0x48
#define INFO             0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L	 0x03  // data
#define AK8963_XOUT_H	 0x04
#define AK8963_YOUT_L	 0x05
#define AK8963_YOUT_H	 0x06
#define AK8963_ZOUT_L	 0x07
#define AK8963_ZOUT_H	 0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define MPU9250_SELF_TEST_X_GYRO 0x00                  
#define MPU9250_SELF_TEST_Y_GYRO 0x01                                                                          
#define MPU9250_SELF_TEST_Z_GYRO 0x02

#define MPU9250_SELF_TEST_X_ACCEL 0x0D
#define MPU9250_SELF_TEST_Y_ACCEL 0x0E    
#define MPU9250_SELF_TEST_Z_ACCEL 0x0F

#define MPU9250_SELF_TEST_A      0x10

#define MPU9250_XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define MPU9250_XG_OFFSET_L      0x14
#define MPU9250_YG_OFFSET_H      0x15
#define MPU9250_YG_OFFSET_L      0x16
#define MPU9250_ZG_OFFSET_H      0x17
#define MPU9250_ZG_OFFSET_L      0x18
#define MPU9250_SMPLRT_DIV       0x19
#define MPU9250_CONFIG           0x1A
#define MPU9250_GYRO_CONFIG      0x1B
#define MPU9250_ACCEL_CONFIG     0x1C
#define MPU9250_ACCEL_CONFIG2    0x1D
#define MPU9250_LP_ACCEL_ODR     0x1E   
#define MPU9250_WOM_THR          0x1F   

#define MPU9250_MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define MPU9250_ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define MPU9250_ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define MPU9250_FIFO_EN          0x23
#define MPU9250_I2C_MST_CTRL     0x24   
#define MPU9250_I2C_SLV0_ADDR    0x25
#define MPU9250_I2C_SLV0_REG     0x26
#define MPU9250_I2C_SLV0_CTRL    0x27
#define MPU9250_I2C_SLV1_ADDR    0x28
#define MPU9250_I2C_SLV1_REG     0x29
#define MPU9250_I2C_SLV1_CTRL    0x2A
#define MPU9250_I2C_SLV2_ADDR    0x2B
#define MPU9250_I2C_SLV2_REG     0x2C
#define MPU9250_I2C_SLV2_CTRL    0x2D
#define MPU9250_I2C_SLV3_ADDR    0x2E
#define MPU9250_I2C_SLV3_REG     0x2F
#define MPU9250_I2C_SLV3_CTRL    0x30
#define MPU9250_I2C_SLV4_ADDR    0x31
#define MPU9250_I2C_SLV4_REG     0x32
#define MPU9250_I2C_SLV4_DO      0x33
#define MPU9250_I2C_SLV4_CTRL    0x34
#define MPU9250_I2C_SLV4_DI      0x35
#define MPU9250_I2C_MST_STATUS   0x36
#define MPU9250_INT_PIN_CFG      0x37
#define MPU9250_INT_ENABLE       0x38
#define MPU9250_DMP_INT_STATUS   0x39  // Check DMP interrupt
#define MPU9250_INT_STATUS       0x3A
#define MPU9250_ACCEL_XOUT_H     0x3B
#define MPU9250_ACCEL_XOUT_L     0x3C
#define MPU9250_ACCEL_YOUT_H     0x3D
#define MPU9250_ACCEL_YOUT_L     0x3E
#define MPU9250_ACCEL_ZOUT_H     0x3F
#define MPU9250_ACCEL_ZOUT_L     0x40
#define MPU9250_TEMP_OUT_H       0x41
#define MPU9250_TEMP_OUT_L       0x42
#define MPU9250_GYRO_XOUT_H      0x43
#define MPU9250_GYRO_XOUT_L      0x44
#define MPU9250_GYRO_YOUT_H      0x45
#define MPU9250_GYRO_YOUT_L      0x46
#define MPU9250_GYRO_ZOUT_H      0x47
#define MPU9250_GYRO_ZOUT_L      0x48
#define MPU9250_EXT_SENS_DATA_00 0x49
#define MPU9250_EXT_SENS_DATA_01 0x4A
#define MPU9250_EXT_SENS_DATA_02 0x4B
#define MPU9250_EXT_SENS_DATA_03 0x4C
#define MPU9250_EXT_SENS_DATA_04 0x4D
#define MPU9250_EXT_SENS_DATA_05 0x4E
#define MPU9250_EXT_SENS_DATA_06 0x4F
#define MPU9250_EXT_SENS_DATA_07 0x50
#define MPU9250_EXT_SENS_DATA_08 0x51
#define MPU9250_EXT_SENS_DATA_09 0x52
#define MPU9250_EXT_SENS_DATA_10 0x53
#define MPU9250_EXT_SENS_DATA_11 0x54
#define MPU9250_EXT_SENS_DATA_12 0x55
#define MPU9250_EXT_SENS_DATA_13 0x56
#define MPU9250_EXT_SENS_DATA_14 0x57
#define MPU9250_EXT_SENS_DATA_15 0x58
#define MPU9250_EXT_SENS_DATA_16 0x59
#define MPU9250_EXT_SENS_DATA_17 0x5A
#define MPU9250_EXT_SENS_DATA_18 0x5B
#define MPU9250_EXT_SENS_DATA_19 0x5C
#define MPU9250_EXT_SENS_DATA_20 0x5D
#define MPU9250_EXT_SENS_DATA_21 0x5E
#define MPU9250_EXT_SENS_DATA_22 0x5F
#define MPU9250_EXT_SENS_DATA_23 0x60
#define MPU9250_MOT_DETECT_STATUS 0x61
#define MPU9250_I2C_SLV0_DO      0x63
#define MPU9250_I2C_SLV1_DO      0x64
#define MPU9250_I2C_SLV2_DO      0x65
#define MPU9250_I2C_SLV3_DO      0x66
#define MPU9250_I2C_MST_DELAY_CTRL 0x67
#define MPU9250_SIGNAL_PATH_RESET  0x68
#define MPU9250_MOT_DETECT_CTRL  0x69
#define MPU9250_USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define MPU9250_PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define MPU9250_PWR_MGMT_2       0x6C
#define MPU9250_DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define MPU9250_DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define MPU9250_DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define MPU9250_DMP_REG_1        0x70
#define MPU9250_DMP_REG_2        0x71 
#define MPU9250_FIFO_COUNTH      0x72
#define MPU9250_FIFO_COUNTL      0x73
#define MPU9250_FIFO_R_W         0x74
#define MPU9250_WHO_AM_I         0x75 // Should return 0x71
#define MPU9250_XA_OFFSET_H      0x77
#define MPU9250_XA_OFFSET_L      0x78
#define MPU9250_YA_OFFSET_H      0x7A
#define MPU9250_YA_OFFSET_L      0x7B
#define MPU9250_ZA_OFFSET_H      0x7D
#define MPU9250_ZA_OFFSET_L      0x7E

// BMP280 registers
#define BMP280_TEMP_XLSB  0xFC
#define BMP280_TEMP_LSB   0xFB
#define BMP280_TEMP_MSB   0xFA
#define BMP280_PRESS_XLSB 0xF9
#define BMP280_PRESS_LSB  0xF8
#define BMP280_PRESS_MSB  0xF7
#define BMP280_CONFIG     0xF5
#define BMP280_CTRL_MEAS  0xF4
#define BMP280_STATUS     0xF3
#define BMP280_RESET      0xE0
#define BMP280_ID         0xD0  // should be 0x58
#define BMP280_CALIB00    0x88

#define MPU9250_ADDRESS 0x68       // MPU9250 address when ADO = 1
#define AK8963_ADDRESS 0x0C        // Address of AK8963 (MPU9250) magnetometer
#define BMP280_ADDRESS 0x77        // Address of BMP280 altimeter 

#define SerialDebug true  // set to true to get Serial output for debugging

// Set initial input parameters
enum MPU9250Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum MPU9250Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum MPU9250Mscale {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};

enum BMP280Posr {
  P_OSR_00 = 0,  // no op
  P_OSR_01,
  P_OSR_02,
  P_OSR_04,
  P_OSR_08,
  P_OSR_16
};

enum BMP280Tosr {
  T_OSR_00 = 0,  // no op
  T_OSR_01,
  T_OSR_02,
  T_OSR_04,
  T_OSR_08,
  T_OSR_16
};

enum BMP280IIRFilter {
  full = 0,  // bandwidth at full sample rate
  BW0_223ODR,
  BW0_092ODR,
  BW0_042ODR,
  BW0_021ODR // bandwidth at 0.021 x sample rate
};

enum BMP280Mode {
  Sleep = 0,
  forced,
  forced2,
  normal
};

enum BMP280SBy {
  t_00_5ms = 0,
  t_62_5ms,
  t_125ms,
  t_250ms,
  t_500ms,
  t_1000ms,
  t_2000ms,
  t_4000ms,
};

// Specify BMP280 configuration
uint8_t BMP280Posr = P_OSR_16, BMP280Tosr = T_OSR_02, BMP280Mode = normal, BMP280IIRFilter = BW0_042ODR, BMP280SBy = t_62_5ms;     // set pressure amd temperature output data rate
// t_fine carries fine temperature as global value for BMP280
int32_t t_fine;

uint8_t MPU9250Gscale = GFS_250DPS;
uint8_t MPU9250Ascale = AFS_2G;
uint8_t MPU9250Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
uint8_t MPU9250Mmode = 0x06;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
float MPU9250aRes, MPU9250gRes, MPU9250mRes;             // scale resolutions per LSB for the sensors

// BMP280 compensation parameters
uint16_t dig_T1, dig_P1;
int16_t  dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

// Pin definitions
int myLed = 27;

int32_t rawPress, rawTemp;   // pressure and temperature raw count output for BMP280
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float magCalibration[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float MPU9250gyroBias[3] = {0, 0, 0}, MPU9250accelBias[3] = {0, 0, 0}, MPU9250magBias[3] = {0, 0, 0};      // Bias corrections for gyro and accelerometer
int16_t tempCount;            // temperature raw count output
float   temperature, altitude;          // Stores the MPU9250 gyro internal chip temperature in degrees Celsius
double BMP280Temperature, BMP280Pressure; // stores MS5637 pressures sensor pressure and temperature
float SelfTest[6];            // holds results of gyro and accelerometer self test

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float pi = 3.14159f;
float GyroMeasError = pi * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = pi * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

uint32_t delt_t = 0, count = 0, sumCount = 0;  // used to control display output rate
float pitch, yaw, roll;
float a12, a22, a31, a32, a33;            // rotation matrix coefficients for Euler angles and gravity components
float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;                         // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest MPU9250 sensor data values 
float lin_ax, lin_ay, lin_az;             // linear acceleration (acceleration with gravity component subtracted)
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method


void setup()
{
//  Wire.begin();
//  TWBR = 12;  // 400 kbit/sec I2C speed for Pro Mini
  // Setup for Master mode, pins 18/19, external pullups, 400kHz for Teensy 3.1
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);
  Serial.begin(38400);
  delay(4000);
  
  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);
  
  delay(1000);

  I2Cscan();// look for I2C devices on the bus
    
  // Read the WHO_AM_I register, this is a good test of communication
  Serial.println("MPU9250 9-axis motion sensor...");
  byte c = readByte(MPU9250_ADDRESS, MPU9250_WHO_AM_I);  // Read WHO_AM_I register for MPU-9250
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);

  delay(1000); 

  if (c == 0x71) // WHO_AM_I should always be 0x68
  {  
    Serial.println("MPU9250 is online...");
    
    MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
    Serial.println("MPU9250 Self Test:");
    Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5],1); Serial.println("% of factory value");
    delay(1000);
    
   // get sensor resolutions, only need to do this once
   MPU9250getAres();
   MPU9250getGres();
   MPU9250getMres();
    
   Serial.println(" Calibrate MPU9250 gyro and accel");
   accelgyrocalMPU9250(MPU9250gyroBias, MPU9250accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
   Serial.println("accel biases (mg)"); Serial.println(1000.*MPU9250accelBias[0]); Serial.println(1000.*MPU9250accelBias[1]); Serial.println(1000.*MPU9250accelBias[2]);
   Serial.println("gyro biases (dps)"); Serial.println(MPU9250gyroBias[0]); Serial.println(MPU9250gyroBias[1]); Serial.println(MPU9250gyroBias[2]);

  delay(1000);  
   
  initMPU9250(); 
  Serial.println("MPU9250 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
  
  // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
  byte d = readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);  // Read WHO_AM_I register for AK8963
  Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);

  delay(1000); 
  
  // Get magnetometer calibration from AK8963 ROM
  initAK8963(magCalibration); Serial.println("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer
  
  magcalMPU9250(MPU9250magBias);
  Serial.println("AK8963 mag biases (mG)"); Serial.println(MPU9250magBias[0]); Serial.println(MPU9250magBias[1]); Serial.println(MPU9250magBias[2]); 
  delay(2000); // add delay to see results before serial spew of data
   
  if(SerialDebug) {
  Serial.print("X-Axis sensitivity adjustment value "); Serial.println(magCalibration[0], 2);
  Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(magCalibration[1], 2);
  Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(magCalibration[2], 2);
  }
  
  delay(1000);  
  
    }
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }
  
  // Read the WHO_AM_I register of the BMP280 this is a good test of communication
  byte f = readByte(BMP280_ADDRESS, BMP280_ID);  // Read WHO_AM_I register for BMP280
  Serial.print("BMP280 "); 
  Serial.print("I AM "); 
  Serial.print(f, HEX); 
  Serial.print(" I should be "); 
  Serial.println(0x58, HEX);
  Serial.println(" ");

  delay(1000); 

  writeByte(BMP280_ADDRESS, BMP280_RESET, 0xB6); // reset BMP280 before initilization
  delay(100);

  BMP280Init(); // Initialize BMP280 altimeter
  Serial.println("BMP280 Calibration coeficients:");
  Serial.print("dig_T1 ="); 
  Serial.println(dig_T1);
  Serial.print("dig_T2 ="); 
  Serial.println(dig_T2);
  Serial.print("dig_T3 ="); 
  Serial.println(dig_T3);
  Serial.print("dig_P1 ="); 
  Serial.println(dig_P1);
  Serial.print("dig_P2 ="); 
  Serial.println(dig_P2);
  Serial.print("dig_P3 ="); 
  Serial.println(dig_P3);
  Serial.print("dig_P4 ="); 
  Serial.println(dig_P4);
  Serial.print("dig_P5 ="); 
  Serial.println(dig_P5);
  Serial.print("dig_P6 ="); 
  Serial.println(dig_P6);
  Serial.print("dig_P7 ="); 
  Serial.println(dig_P7);
  Serial.print("dig_P8 ="); 
  Serial.println(dig_P8);
  Serial.print("dig_P9 ="); 
  Serial.println(dig_P9);
 
}

void loop()
{  
  //MPU9250
  // If intPin goes high, all data registers have new data
  if (readByte(MPU9250_ADDRESS, MPU9250_INT_STATUS) & 0x01) {  // check if data ready interrupt
    MPU9250readAccelData(accelCount);  // Read the x/y/z adc values
 
    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0]*MPU9250aRes - MPU9250accelBias[0];  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1]*MPU9250aRes - MPU9250accelBias[1];   
    az = (float)accelCount[2]*MPU9250aRes - MPU9250accelBias[2];  
   
    MPU9250readGyroData(gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0]*MPU9250gRes;  // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1]*MPU9250gRes;  
    gz = (float)gyroCount[2]*MPU9250gRes;   
  
    MPU9250readMagData(magCount);  // Read the x/y/z adc values
    
    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    mx = (float)magCount[0]*MPU9250mRes*magCalibration[0] - MPU9250magBias[0];  // get actual magnetometer value, this depends on scale being set
    my = (float)magCount[1]*MPU9250mRes*magCalibration[1] - MPU9250magBias[1];  
    mz = (float)magCount[2]*MPU9250mRes*magCalibration[2] - MPU9250magBias[2];   
   }
  
  Now = micros();
  deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;

  sum += deltat; // sum for averaging filter update rate
  sumCount++;
  
  // Sensors x (y)-axis of the accelerometer/gyro is aligned with the y (x)-axis of the magnetometer;
  // the magnetometer z-axis (+ down) is misaligned with z-axis (+ up) of accelerometer and gyro!
  // We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
  // For the MPU9250+MS5637 Mini breakout the +x accel/gyro is North, then -y accel/gyro is East. So if we want te quaternions properly aligned
  // we need to feed into the Madgwick function Ax, -Ay, -Az, Gx, -Gy, -Gz, My, -Mx, and Mz. But because gravity is by convention
  // positive down, we need to invert the accel data, so we pass -Ax, Ay, Az, Gx, -Gy, -Gz, My, -Mx, and Mz into the Madgwick
  // function to get North along the accel +x-axis, East along the accel -y-axis, and Down along the accel -z-axis.
  // This orientation choice can be modified to allow any convenient (non-NED) orientation convention.
  // Pass gyro rate as rad/s
    MadgwickQuaternionUpdate(-ax, ay, az, gx*pi/180.0f, -gy*pi/180.0f, -gz*pi/180.0f,  my,  -mx, mz);
//  MahonyQuaternionUpdate(-ax, ay, az, gx*pi/180.0f, -gy*pi/180.0f, -gz*pi/180.0f,  my,  -mx, mz);

    // Serial print and/or display at 0.5 s rate independent of data rates
    delt_t = millis() - count;
    if (delt_t > 500) { // update LCD once per half-second independent of read rate

    if(SerialDebug) {
     Serial.println("MPU9250: ");
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
    }               
    tempCount = MPU9250readTempData();  // Read the gyro adc values
    temperature = ((float) tempCount) / 333.87 + 21.0; // Gyro chip temperature in degrees Centigrade
   // Print temperature in degrees Centigrade      
   if(SerialDebug) {
     Serial.print("MPU9250 Gyro temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
   }
   
    rawPress =  readBMP280Pressure();
    BMP280Pressure = (float) bmp280_compensate_P(rawPress)/25600.; // Prwssure in mbar
    rawTemp =   readBMP280Temperature();
    BMP280Temperature = (float) bmp280_compensate_T(rawTemp)/100.;

    altitude = 145366.45f*(1.0f - pow((BMP280Pressure/1013.25f), 0.190284f));

    if(SerialDebug) {
      Serial.println("BMP280:");
      Serial.print("Altimeter temperature = "); 
      Serial.print( BMP280Temperature, 2); 
      Serial.println(" C"); // temperature in degrees Celsius
      Serial.print("Altimeter temperature = "); 
      Serial.print(9.*BMP280Temperature/5. + 32., 2); 
      Serial.println(" F"); // temperature in degrees Fahrenheit
      Serial.print("Altimeter pressure = "); 
      Serial.print(BMP280Pressure, 2);  
      Serial.println(" mbar");// pressure in millibar
      Serial.print("Altitude = "); 
      Serial.print(altitude, 2); 
      Serial.println(" feet");
      Serial.println(" ");
    }
      
   // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
  // In this coordinate system, the positive z-axis is down toward Earth. 
  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
  // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    //Software AHRS:
 //   yaw   = atan2f(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
 //   pitch = -asinf(2.0f * (q[1] * q[3] - q[0] * q[2]));
 //   roll  = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
 //   pitch *= 180.0f / PI;
 //   yaw   *= 180.0f / PI; 
 //   yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
 //   if(yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360
 //   roll  *= 180.0f / PI;
    a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
    a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
    a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
    a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
    a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
    pitch = -asinf(a32);
    roll  = atan2f(a31, a33);
    yaw   = atan2f(a12, a22);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI; 
    yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    if(yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360
    roll  *= 180.0f / PI;
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
    Serial.print(-a31*1000, 2);
    Serial.print(", ");
    Serial.print(-a32*1000, 2);
    Serial.print(", ");
    Serial.print(a33*1000, 2);  Serial.println(" mg");
    Serial.print("Lin_ax, Lin_ay, Lin_az: ");
    Serial.print(lin_ax*1000, 2);
    Serial.print(", ");
    Serial.print(lin_ay*1000, 2);
    Serial.print(", ");
    Serial.print(lin_az*1000, 2);  Serial.println(" mg");
    
    Serial.print("rate = "); Serial.print((float)sumCount/sum, 2); Serial.println(" Hz");
    }

    digitalWrite(myLed, !digitalRead(myLed));
    count = millis(); 
    sumCount = 0;
    sum = 0;    
    }

}

//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

void MPU9250getMres() {
  switch (MPU9250Mscale)
  {
 	// Possible magnetometer scales (and their register bit settings) are:
	// 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
          MPU9250mRes = 10.*4912./8190.; // Proper scale to return milliGauss
          break;
    case MFS_16BITS:
          MPU9250mRes = 10.*4912./32760.0; // Proper scale to return milliGauss
          break;
  }
}

void MPU9250getGres() {
  switch (MPU9250Gscale)
  {
 	// Possible gyro scales (and their register bit settings) are:
	// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
          MPU9250gRes = 250.0/32768.0;
          break;
    case GFS_500DPS:
          MPU9250gRes = 500.0/32768.0;
          break;
    case GFS_1000DPS:
          MPU9250gRes = 1000.0/32768.0;
          break;
    case GFS_2000DPS:
          MPU9250gRes = 2000.0/32768.0;
          break;
  }
}

void MPU9250getAres() {
  switch (MPU9250Ascale)
  {
 	// Possible accelerometer scales (and their register bit settings) are:
	// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
          MPU9250aRes = 2.0/32768.0;
          break;
    case AFS_4G:
          MPU9250aRes = 4.0/32768.0;
          break;
    case AFS_8G:
          MPU9250aRes = 8.0/32768.0;
          break;
    case AFS_16G:
          MPU9250aRes = 16.0/32768.0;
          break;
  }
}

void MPU9250readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(MPU9250_ADDRESS, MPU9250_ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}


void MPU9250readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(MPU9250_ADDRESS, MPU9250_GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}

void MPU9250readMagData(int16_t * destination)
{
  uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  if(readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
  readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
  uint8_t c = rawData[6]; // End data read by reading ST2 register
    if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
    destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
    destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
   }
  }
}

int16_t MPU9250readTempData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readBytes(MPU9250_ADDRESS,MPU9250_TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array 
  return ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}
       
void initAK8963(float * destination)
{
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
  delay(10);
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  delay(10);
  readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
  destination[0] =  (float)(rawData[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawData[1] - 128)/256. + 1.;  
  destination[2] =  (float)(rawData[2] - 128)/256. + 1.; 
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
  delay(10);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeByte(AK8963_ADDRESS, AK8963_CNTL, MPU9250Mscale << 4 | MPU9250Mmode); // Set magnetometer data resolution and sample ODR
  delay(10);
}


void initMPU9250()
{  
 // wake up device
  writeByte(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
  delay(100); // Wait for all registers to reset 

 // get stable time source
  writeByte(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  delay(200); 
  
 // Configure Gyro and Thermometer
 // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
 // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
 // be higher than 1 / 0.0059 = 170 Hz
 // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
 // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  writeByte(MPU9250_ADDRESS, MPU9250_CONFIG, 0x03);  

 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(MPU9250_ADDRESS, MPU9250_SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate 
                                    // determined inset in CONFIG above
 
 // Set gyroscope full scale range
 // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = readByte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG);
//  writeRegister(GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
  writeByte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, c & ~0x03); // Clear Fchoice bits [1:0] 
  writeByte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, c & ~0x18); // Clear GFS bits [4:3]
  writeByte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, c | MPU9250Gscale << 3); // Set full scale range for the gyro
 // writeRegister(GYRO_CONFIG, c | 0x00); // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  
 // Set accelerometer full-scale range configuration
  c = readByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG);
//  writeRegister(ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
  writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, c | MPU9250Ascale << 3); // Set full scale range for the accelerometer 

 // Set accelerometer sample rate configuration
 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = readByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG2);
  writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG2, c & ~0x0F); // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
  writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG2, c | 0x03); // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz

 // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
 // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
  // can join the I2C bus and all can be controlled by the Arduino as master
   writeByte(MPU9250_ADDRESS, MPU9250_INT_PIN_CFG, 0x22);    
   writeByte(MPU9250_ADDRESS, MPU9250_INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
   delay(100);
}


// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void accelgyrocalMPU9250(float * dest1, float * dest2)
{  
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  
 // reset device
  writeByte(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);
   
 // get stable time source; Auto select clock source to be PLL gyroscope reference if ready 
 // else use the internal oscillator, bits 2:0 = 001
  writeByte(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x01);  
  writeByte(MPU9250_ADDRESS, MPU9250_PWR_MGMT_2, 0x00);
  delay(200);                                    

// Configure device for bias calculation
  writeByte(MPU9250_ADDRESS, MPU9250_INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(MPU9250_ADDRESS, MPU9250_FIFO_EN, 0x00);      // Disable FIFO
  writeByte(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(MPU9250_ADDRESS, MPU9250_I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(MPU9250_ADDRESS, MPU9250_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(MPU9250_ADDRESS, MPU9250_USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);
  
// Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte(MPU9250_ADDRESS, MPU9250_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeByte(MPU9250_ADDRESS, MPU9250_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
 
  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

// Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(MPU9250_ADDRESS, MPU9250_USER_CTRL, 0x40);   // Enable FIFO  
  writeByte(MPU9250_ADDRESS, MPU9250_FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

// At end of sample accumulation, turn off FIFO sensor read
  writeByte(MPU9250_ADDRESS, MPU9250_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(MPU9250_ADDRESS, MPU9250_FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
  
  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(MPU9250_ADDRESS, MPU9250_FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
    
    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
            
}
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;
    
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}
   
// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;
  
// Push gyro biases to hardware registers
  writeByte(MPU9250_ADDRESS, MPU9250_XG_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, MPU9250_XG_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, MPU9250_YG_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, MPU9250_YG_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, MPU9250_ZG_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, MPU9250_ZG_OFFSET_L, data[5]);
  
// Output scaled gyro biases for display in the main program
  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;  
  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readBytes(MPU9250_ADDRESS, MPU9250_XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(MPU9250_ADDRESS, MPU9250_YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(MPU9250_ADDRESS, MPU9250_ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  
  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
  
  for(ii = 0; ii < 3; ii++) {
    if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }
  
  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);
  
  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers
 
// Apparently this is not working for the acceleration biases in the MPU-9250
// Are we handling the temperature correction bit properly?
// Push accelerometer biases to hardware registers
/*  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
*/
// Output scaled accelerometer biases for display in the main program
   dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
   dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
   dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}


void magcalMPU9250(float * dest1) 
{
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0};
  int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};
 
  Serial.println("Mag Calibration: Wave device in a figure eight until done!");
  delay(4000);
  
   sample_count = 64;
   for(ii = 0; ii < sample_count; ii++) {
    MPU9250readMagData(mag_temp);  // Read the mag data   
    for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
   }

//    Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
//    Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
//    Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
    
    dest1[0] = (float) mag_bias[0]*MPU9250mRes*magCalibration[0];  // save mag biases in G for main program
    dest1[1] = (float) mag_bias[1]*MPU9250mRes*magCalibration[1];   
    dest1[2] = (float) mag_bias[2]*MPU9250mRes*magCalibration[2];          

   Serial.println("Mag Calibration done!");
}



// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
   uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
   uint8_t selfTest[6];
   int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
   float factoryTrim[6];
   uint8_t FS = 0;
   
  writeByte(MPU9250_ADDRESS, MPU9250_SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
  writeByte(MPU9250_ADDRESS, MPU9250_CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  writeByte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, 1<<FS);  // Set full scale range for the gyro to 250 dps
  writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 1<<FS); // Set full scale range for the accelerometer to 2 g

  for( int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer
  
  readBytes(MPU9250_ADDRESS, MPU9250_ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
  aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  
    readBytes(MPU9250_ADDRESS, MPU9250_GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
  gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  }
  
  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
  aAvg[ii] /= 200;
  gAvg[ii] /= 200;
  }
  
// Configure the accelerometer for self-test
   writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
   writeByte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
   delay(25);  // Delay a while to let the device stabilize

  for( uint16_t ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer
  
  readBytes(MPU9250_ADDRESS, MPU9250_ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  
    readBytes(MPU9250_ADDRESS, MPU9250_GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  }
  
  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
  aSTAvg[ii] /= 200;
  gSTAvg[ii] /= 200;
  }   
  
 // Configure the gyro and accelerometer for normal operation
   writeByte(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 0x00);  
   writeByte(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG,  0x00);  
   delay(25);  // Delay a while to let the device stabilize
   
   // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
   selfTest[0] = readByte(MPU9250_ADDRESS, MPU9250_SELF_TEST_X_ACCEL); // X-axis accel self-test results
   selfTest[1] = readByte(MPU9250_ADDRESS, MPU9250_SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
   selfTest[2] = readByte(MPU9250_ADDRESS, MPU9250_SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
   selfTest[3] = readByte(MPU9250_ADDRESS, MPU9250_SELF_TEST_X_GYRO);  // X-axis gyro self-test results
   selfTest[4] = readByte(MPU9250_ADDRESS, MPU9250_SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
   selfTest[5] = readByte(MPU9250_ADDRESS, MPU9250_SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

  // Retrieve factory self-test value from self-test code reads
   factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
   factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
   factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
   factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
   factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
   factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation
 
 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
 // To get percent, must multiply by 100
   for (int i = 0; i < 3; i++) {
     destination[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i] - 100.;   // Report percent differences
     destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3] - 100.; // Report percent differences
   }
   
}


// Returns temperature in DegC, resolution is 0.01 DegC. Output value of
// “5123” equals 51.23 DegC.
int32_t bmp280_compensate_T(int32_t adc_T)
{
  int32_t var1, var2, T;
  var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
  t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;
  return T;
}


// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8
//fractional bits).
//Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
uint32_t bmp280_compensate_P(int32_t adc_P)
{
  long long var1, var2, p;
  var1 = ((long long)t_fine) - 128000;
  var2 = var1 * var1 * (long long)dig_P6;
  var2 = var2 + ((var1*(long long)dig_P5)<<17);
  var2 = var2 + (((long long)dig_P4)<<35);
  var1 = ((var1 * var1 * (long long)dig_P3)>>8) + ((var1 * (long long)dig_P2)<<12);
  var1 = (((((long long)1)<<47)+var1))*((long long)dig_P1)>>33;
  if(var1 == 0)
  {
    return 0;
    // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p<<31) - var2)*3125)/var1;
  var1 = (((long long)dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((long long)dig_P8) * p)>> 19;
  p = ((p + var1 + var2) >> 8) + (((long long)dig_P7)<<4);
  return (uint32_t)p;
}

int32_t readBMP280Temperature()
{
  uint8_t rawData[3];  // 20-bit pressure register data stored here
  readBytes(BMP280_ADDRESS, BMP280_TEMP_MSB, 3, &rawData[0]);  
  return (int32_t) (((int32_t) rawData[0] << 16 | (int32_t) rawData[1] << 8 | rawData[2]) >> 4);
}

int32_t readBMP280Pressure()
{
  uint8_t rawData[3];  // 20-bit pressure register data stored here
  readBytes(BMP280_ADDRESS, BMP280_PRESS_MSB, 3, &rawData[0]);  
  return (int32_t) (((int32_t) rawData[0] << 16 | (int32_t) rawData[1] << 8 | rawData[2]) >> 4);
}


void BMP280Init()
{
  // Configure the BMP280
  // Set T and P oversampling rates and sensor mode
  writeByte(BMP280_ADDRESS, BMP280_CTRL_MEAS, BMP280Tosr << 5 | BMP280Posr << 2 | BMP280Mode);
  // Set standby time interval in normal mode and bandwidth
  writeByte(BMP280_ADDRESS, BMP280_CONFIG, BMP280SBy << 5 | BMP280IIRFilter << 2);
  // Read and store calibration data
  uint8_t calib[24];
  readBytes(BMP280_ADDRESS, BMP280_CALIB00, 24, &calib[0]);
  dig_T1 = (uint16_t)(((uint16_t) calib[1] << 8) | calib[0]);
  dig_T2 = ( int16_t)((( int16_t) calib[3] << 8) | calib[2]);
  dig_T3 = ( int16_t)((( int16_t) calib[5] << 8) | calib[4]);
  dig_P1 = (uint16_t)(((uint16_t) calib[7] << 8) | calib[6]);
  dig_P2 = ( int16_t)((( int16_t) calib[9] << 8) | calib[8]);
  dig_P3 = ( int16_t)((( int16_t) calib[11] << 8) | calib[10]);
  dig_P4 = ( int16_t)((( int16_t) calib[13] << 8) | calib[12]);
  dig_P5 = ( int16_t)((( int16_t) calib[15] << 8) | calib[14]);
  dig_P6 = ( int16_t)((( int16_t) calib[17] << 8) | calib[16]);
  dig_P7 = ( int16_t)((( int16_t) calib[19] << 8) | calib[18]);
  dig_P8 = ( int16_t)((( int16_t) calib[21] << 8) | calib[20]);
  dig_P9 = ( int16_t)((( int16_t) calib[23] << 8) | calib[22]);
}

// I2C scan function

void I2Cscan()
{
// scan for i2c devices
  byte error, address;
  uint16_t nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
    
}


// I2C read/write functions for the MPU9250 and AK8963 sensors

        void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	Wire.beginTransmission(address);  // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.write(data);                 // Put data in Tx buffer
	Wire.endTransmission();           // Send the Tx buffer
}

        uint8_t readByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data; // `data` will store the register data	 
	Wire.beginTransmission(address);         // Initialize the Tx buffer
	Wire.write(subAddress);	                 // Put slave register address in Tx buffer
	Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.requestFrom(address, 1);  // Read one byte from slave register address 
	Wire.requestFrom(address, (size_t) 1);   // Read one byte from slave register address 
	data = Wire.read();                      // Fill Rx buffer with result
	return data;                             // Return data read from slave register
}

        void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
	Wire.beginTransmission(address);   // Initialize the Tx buffer
	Wire.write(subAddress);            // Put slave register address in Tx buffer
	Wire.endTransmission(I2C_NOSTOP);  // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
//        Wire.requestFrom(address, count);  // Read bytes from slave register address 
        Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address 
	while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}

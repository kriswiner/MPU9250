/* MPU9250 Basic Example Code
 by: Kris Winer
 date: April 1, 2014
 license: Beerware - Use this code however you'd like. If you 
 find it useful you can buy me a beer some time.
 
 Demonstrate basic MPU-9250 functionality including parameterizing the register addresses, initializing the sensor, 
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. Added display functions to 
 allow display to on breadboard monitor. Addition of 9 DoF sensor fusion using open source Madgwick and 
 Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini and the Teensy 3.1.
 
 SDA and SCL should have external pull-up resistors (to 3.3V).
 10k resistors are on the EMSENSR-9250 breakout board.
 
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
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

// Using NOKIA 5110 monochrome 84 x 48 pixel display
// pin 9 - Serial clock out (SCLK)
// pin 8 - Serial data out (DIN)
// pin 7 - Data/Command select (D/C)
// pin 5 - LCD chip select (CS)
// pin 6 - LCD reset (RST)
Adafruit_PCD8544 display = Adafruit_PCD8544(9, 8, 7, 5, 6);

// See also MPU-9250 Register Map and Descriptions, Revision 4.0, RM-MPU-9250A-00, Rev. 1.4, 9/9/2013 for registers not listed in 
// above document; the MPU9250 and MPU9150 are virtually identical but the latter has a different register map
//
//Magnetometer Registers
#define AK8963_ADDRESS   0x0C
#define AK8963_WHO_AM_I  0x00 // should return 0x48
#define AK8963_INFO      0x01
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

#define SELF_TEST_X_GYRO 0x00                  
#define SELF_TEST_Y_GYRO 0x01                                                                          
#define SELF_TEST_Z_GYRO 0x02

/*#define X_FINE_GAIN      0x03 // [7:0] fine gain
#define Y_FINE_GAIN      0x04
#define Z_FINE_GAIN      0x05
#define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC   0x07
#define YA_OFFSET_H      0x08
#define YA_OFFSET_L_TC   0x09
#define ZA_OFFSET_H      0x0A
#define ZA_OFFSET_L_TC   0x0B */

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E    
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A      0x10

#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E   
#define WOM_THR          0x1F   

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24   
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71 
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E


// Standard 7-bit I2C slave address is 1100000 = 0x60, 8-bit read address is 0xC1, 8-bit write is 0xC0
#define MPL3115A2_ADDRESS 0x60  // SA0 is high, 0x1C if low

// Register defines courtesy A. Weiss and Nathan Seidle, SparkFun Electronics
#define MPL3115A2_STATUS     0x00
#define MPL3115A2_OUT_P_MSB  0x01
#define MPL3115A2_OUT_P_CSB  0x02
#define MPL3115A2_OUT_P_LSB  0x03
#define MPL3115A2_OUT_T_MSB  0x04
#define MPL3115A2_OUT_T_LSB  0x05
#define MPL3115A2_DR_STATUS  0x06
#define MPL3115A2_OUT_P_DELTA_MSB  0x07
#define MPL3115A2_OUT_P_DELTA_CSB  0x08
#define MPL3115A2_OUT_P_DELTA_LSB  0x09
#define MPL3115A2_OUT_T_DELTA_MSB  0x0A
#define MPL3115A2_OUT_T_DELTA_LSB  0x0B
#define MPL3115A2_WHO_AM_I   0x0C
#define MPL3115A2_F_STATUS   0x0D
#define MPL3115A2_F_DATA     0x0E
#define MPL3115A2_F_SETUP    0x0F
#define MPL3115A2_TIME_DLY   0x10
#define MPL3115A2_SYSMOD     0x11
#define MPL3115A2_INT_SOURCE 0x12
#define MPL3115A2_PT_DATA_CFG 0x13
#define MPL3115A2_BAR_IN_MSB 0x14 // Set at factory to equivalent sea level pressure for measurement location, generally no need to change
#define MPL3115A2_BAR_IN_LSB 0x15 // Set at factory to equivalent sea level pressure for measurement location, generally no need to change
#define MPL3115A2_P_TGT_MSB  0x16
#define MPL3115A2_P_TGT_LSB  0x17
#define MPL3115A2_T_TGT      0x18
#define MPL3115A2_P_WND_MSB  0x19
#define MPL3115A2_P_WND_LSB  0x1A
#define MPL3115A2_T_WND      0x1B
#define MPL3115A2_P_MIN_MSB  0x1C
#define MPL3115A2_P_MIN_CSB  0x1D
#define MPL3115A2_P_MIN_LSB  0x1E
#define MPL3115A2_T_MIN_MSB  0x1F
#define MPL3115A2_T_MIN_LSB  0x20
#define MPL3115A2_P_MAX_MSB  0x21
#define MPL3115A2_P_MAX_CSB  0x22
#define MPL3115A2_P_MAX_LSB  0x23
#define MPL3115A2_T_MAX_MSB  0x24
#define MPL3115A2_T_MAX_LSB  0x25
#define MPL3115A2_CTRL_REG1  0x26
#define MPL3115A2_CTRL_REG2  0x27
#define MPL3115A2_CTRL_REG3  0x28
#define MPL3115A2_CTRL_REG4  0x29
#define MPL3115A2_CTRL_REG5  0x2A
#define MPL3115A2_OFF_P      0x2B
#define MPL3115A2_OFF_T      0x2C
#define MPL3115A2_OFF_H      0x2D

// Using the Teensy 3.1 Mini add-on shield, ADO is set to 1 
// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
#define ADO 1
#if ADO
#define MPU9250_ADDRESS 0x69  // Device address when ADO = 1
#define AK8963_ADDRESS 0x0C   //  Address of magnetometer
#else
#define MPU9250_ADDRESS 0x68  // Device address when ADO = 0
#define AK8963_ADDRESS 0x0C   //  Address of magnetometer
#endif  

#define AHRS true         // set to false for basic data read
#define SerialDebug true   // set to true to get Serial output for debugging

// Set initial input parameters
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Mscale {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};

// Specify sensor full scale
uint8_t Gscale = GFS_250DPS;
uint8_t Ascale = AFS_2G;
uint8_t Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x06;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors


enum SAMPLERATE {
  OS_6ms = 0, // 6 ms is minimum oversampling interval, corresponds to an oversample ration of 2^0 = 1 
  OS_10ms,
  OS_18ms,
  OS_34ms,
  OS_66ms,
  OS_130ms, // 130 ms oversampling interval, 2^5 = 32 oversample ratio
  OS_258ms,
  OS_512ms
};

enum ST_VALUE {
  ATS_1s = 0, // 6 ms is minimum oversampling interval, corresponds to an oversample ration of 2^0 = 1 
  ATS_2s,
  ATS_4s,
  ATS_8s,
  ATS_16s,
  ATS_32s,
  ATS_64s, // 2^6 = 64 s interval between up to 32 FIFO samples for half an hour of data logging
  ATS_128s,
  ATS_256s,
  ATS_512s,
  ATS_1024s,
  ATS_2048s,
  ATS_4096s,
  ATS_8192s,
  ATS_16384s,
  ATS_32768s  // 2^15 = 32768 s interval between up to 32 FIFO samples = 12 days of data logging!
};

uint8_t SAMPLERATE = OS_130ms;
uint8_t ST_VALUE = ATS_4s;
int AltimeterMode = 0; // use to choose between altimeter and barometer modes for FIFO data

// Define device outputs
float altitude = 0.;
float pressure = 0.;

// Pin definitions
int INTGX = 10;  // Interrupt for MPU9250 accelerometer/gyro
int INTP1 = 9;   // Interrupt 1 for MPL3115A2 pressure sensor
int INTP2 = 8;   // Interrupt 2 for MPL3115A2 pressure sensor
int myLed = 13;  // Teensy led is on pin 13

int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0}; // Factory mag calibration and mag bias
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};      // Bias corrections for gyro and accelerometer
int16_t tempCount;      // temperature raw count output
float   temperature, Temperature;    // Stores the real internal chip temperature in degrees Celsius
float   SelfTest[6];    // holds results of gyro and accelerometer self test

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
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

uint32_t delt_t = 0; // used to control display output rate
uint32_t count = 0, sumCount = 0; // used to control display output rate
float pitch, yaw, roll;
float deltat = 0.0f, sum = 0.0f;        // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;        // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method


void setup()
{
  // Setup for Master mode, pins 16/18, external pullups, 400kHz
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);
  delay(1000);
  Serial.begin(38400);
  
  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(INTGX,  INPUT);
  pinMode(INTP1,  INPUT);
  pinMode(INTP2,  INPUT);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);
  
  display.begin(); // Initialize the display
  display.setContrast(58); // Set the contrast
  
// Start device display with ID of sensor
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0); display.print("MPU9250");
  display.setTextSize(1);
  display.setCursor(0, 20); display.print("9-DOF 16-bit");
  display.setCursor(0, 30); display.print("motion sensor");
  display.setCursor(20,40); display.print("60 ug LSB");
  display.display();
  delay(1000);

// Set up for data display
  display.setTextSize(1); // Set text size to normal, 2 is twice normal etc.
  display.setTextColor(BLACK); // Set pixel color; 1 on the monochrome screen
  display.clearDisplay();   // clears the screen and buffer

  // Read the WHO_AM_I register, this is a good test of communication
  Serial.println("MPU9250 + MPL3115A2 devices...");
  byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);
  display.setCursor(20,0); display.print("MPU9250");
  display.setCursor(0,10); display.print("I AM");
  display.setCursor(0,20); display.print(c, HEX);  
  display.setCursor(0,30); display.print("I Should Be");
  display.setCursor(0,40); display.print(0x71, HEX); 
  display.display();
  delay(5000); 

  if (c == 0x71) // WHO_AM_I should always be 0x68
  {  
    Serial.println("MPU9250 is online...");
    
    MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
    Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5],1); Serial.println("% of factory value");
    delay(5000);
    
  calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
  display.clearDisplay();
     
  display.setCursor(0, 0); display.print("MPU9250 bias");
  display.setCursor(0, 8); display.print(" x   y   z  ");

  display.setCursor(0,  16); display.print((int)(1000*accelBias[0])); 
  display.setCursor(24, 16); display.print((int)(1000*accelBias[1])); 
  display.setCursor(48, 16); display.print((int)(1000*accelBias[2])); 
  display.setCursor(72, 16); display.print("mg");
    
  display.setCursor(0,  24); display.print(gyroBias[0], 1); 
  display.setCursor(24, 24); display.print(gyroBias[1], 1); 
  display.setCursor(48, 24); display.print(gyroBias[2], 1); 
  display.setCursor(66, 24); display.print("o/s");   
 
  display.display();
  delay(1000); 
  
  initMPU9250(); 
  Serial.println("MPU9250 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
  
  // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
  byte d = readByte(AK8963_ADDRESS, AK8963_WHO_AM_I);  // Read WHO_AM_I register for AK8963
  Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);
  display.clearDisplay();
  display.setCursor(20,0); display.print("AK8963");
  display.setCursor(0,10); display.print("I AM");
  display.setCursor(0,20); display.print(d, HEX);  
  display.setCursor(0,30); display.print("I Should Be");
  display.setCursor(0,40); display.print(0x48, HEX);  
  display.display();
  delay(1000); 
  
  // Get magnetometer calibration from AK8963 ROM
  initAK8963(magCalibration); Serial.println("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer
  
  if(SerialDebug) {
//  Serial.println("Calibration values: ");
  Serial.print("X-Axis sensitivity adjustment value "); Serial.println(magCalibration[0], 2);
  Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(magCalibration[1], 2);
  Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(magCalibration[2], 2);
  }
  
  display.clearDisplay();
  display.setCursor(20,0); display.print("AK8963");
  display.setCursor(0,10); display.print("ASAX "); display.setCursor(50,10); display.print(magCalibration[0], 2);
  display.setCursor(0,20); display.print("ASAY "); display.setCursor(50,20); display.print(magCalibration[1], 2);
  display.setCursor(0,30); display.print("ASAZ "); display.setCursor(50,30); display.print(magCalibration[2], 2);
  display.display();
  delay(1000);  
  }
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }
  
    
  // Read the WHO_AM_I register of the MPL3115A2, this is a good test of communication
  c = readByte(MPL3115A2_ADDRESS, MPL3115A2_WHO_AM_I);  // Read WHO_AM_I register
  Serial.print("MPL3115A2 WHO_AM_I returned: 0x");
  Serial.println(c, HEX);
  Serial.println("Should be 0xC4");
  Serial.println();
  display.setCursor(0, 0); display.print("MPL3115A2");
  display.setCursor(0,10); display.print("I AM");
  display.setCursor(0,20); display.print(c, HEX);  
  display.setCursor(0,30); display.print("I Should Be");
  display.setCursor(0,40); display.print(0xC4, HEX); 
  display.display();
  delay(1000); 
  
    if (c == 0xC4) // WHO_AM_I should always be 0xC4
  {  
    
    MPL3115A2Reset();                // Start off by resetting all registers to the default
    initRealTimeMPL3115A2();         // initialize the accelerometer for realtime data acquisition if communication is OK
    MPL3115A2SampleRate(SAMPLERATE); // Set oversampling ratio
    Serial.print("Oversampling Ratio is "); Serial.println(1<<SAMPLERATE);  
    MPL3115A2enableEventflags();     // Set data ready enable
    Serial.println("MPL3115A2 event flags enabled...");

    display.clearDisplay();   // clears the screen and buffer
    display.setCursor(0, 0); display.print("MPL3115A2");
    display.setCursor(0,10); display.print("OSR "); display.print(1<<SAMPLERATE);  
    display.setCursor(0,20); display.print("Event flags");
    display.setCursor(0,30); display.print("enabled...");
    display.display();
    delay(1000);
  }
  else
  {
    Serial.print("Could not connect to MPL3115A2: 0x");
    Serial.println(c, HEX);
    display.clearDisplay();   // clears the screen and buffer
    display.setCursor(0, 0); display.print("MPL3115A2");
    display.setCursor(0,10); display.print("Error! on 0x"); display.print(c, HEX);
    display.display();
    
    while(1) ; // Loop forever if communication doesn't happen
  }
}

void loop()
{  
  // If intPin goes high, all data registers have new data
  if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt
    readAccelData(accelCount);  // Read the x/y/z adc values
    getAres();
    
    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1]*aRes - accelBias[1];   
    az = (float)accelCount[2]*aRes - accelBias[2];  
   
    readGyroData(gyroCount);  // Read the x/y/z adc values
    getGres();
 
    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0]*gRes;  // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1]*gRes;  
    gz = (float)gyroCount[2]*gRes;   
  
    readMagData(magCount);  // Read the x/y/z adc values
    getMres();

    magbias[0] = +220. ;  // User environmental x-axis correction in milliGauss, should be automatically calculated
    magbias[1] = -620.;  // User environmental y-axis correction in milliGauss
    magbias[2] = +240.;  // User environmental z-axis correction in milliGauss
    
    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    mx = (float)magCount[0]*mRes*magCalibration[0] - magbias[0];  // get actual magnetometer value, this depends on scale being set
    my = (float)magCount[1]*mRes*magCalibration[1] - magbias[1];  
    mz = (float)magCount[2]*mRes*magCalibration[2] - magbias[2];   
  }
  
  Now = micros();
  deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
  lastUpdate = Now;

  sum += deltat; // sum for averaging filter update rate
  sumCount++;
  
  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
  // the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
  // We have to make some allowance for this orientationmismatch in feeding the output to the quaternion filter.
  // For the MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
  // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
  // This is ok by aircraft orientation standards!  
  // Pass gyro rate as rad/s
  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
//  MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);


    if (!AHRS) {
    delt_t = millis() - count;
    if(delt_t > 500) {

    if(SerialDebug) {
    // Print acceleration values in milligs!
    Serial.print("X-acceleration: "); Serial.print(1000*ax); Serial.print(" mg ");
    Serial.print("Y-acceleration: "); Serial.print(1000*ay); Serial.print(" mg ");
    Serial.print("Z-acceleration: "); Serial.print(1000*az); Serial.println(" mg ");
 
    // Print gyro values in degree/sec
    Serial.print("X-gyro rate: "); Serial.print(gx, 3); Serial.print(" degrees/sec "); 
    Serial.print("Y-gyro rate: "); Serial.print(gy, 3); Serial.print(" degrees/sec "); 
    Serial.print("Z-gyro rate: "); Serial.print(gz, 3); Serial.println(" degrees/sec"); 
    
    // Print mag values in degree/sec
    Serial.print("X-mag field: "); Serial.print(mx); Serial.print(" mG "); 
    Serial.print("Y-mag field: "); Serial.print(my); Serial.print(" mG "); 
    Serial.print("Z-mag field: "); Serial.print(mz); Serial.println(" mG"); 
 
    tempCount = readTempData();  // Read the adc values
    Temperature = ((float) tempCount) / 333.87 + 21.0; // Temperature in degrees Centigrade
   // Print temperature in degrees Centigrade      
    Serial.print("Temperature is ");  Serial.print(Temperature, 2);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
    }
   
    display.clearDisplay();     
    display.setCursor(0, 0); display.print("MPU9250/AK8963");
    display.setCursor(0, 8); display.print(" x   y   z  ");

    display.setCursor(0,  16); display.print((int)(1000*ax)); 
    display.setCursor(24, 16); display.print((int)(1000*ay)); 
    display.setCursor(48, 16); display.print((int)(1000*az)); 
    display.setCursor(72, 16); display.print("mg");
    
    display.setCursor(0,  24); display.print((int)(gx)); 
    display.setCursor(24, 24); display.print((int)(gy)); 
    display.setCursor(48, 24); display.print((int)(gz)); 
    display.setCursor(66, 24); display.print("o/s");    
        
    display.setCursor(0,  32); display.print((int)(mx)); 
    display.setCursor(24, 32); display.print((int)(my)); 
    display.setCursor(48, 32); display.print((int)(mz)); 
    display.setCursor(72, 32); display.print("mG");   
   
    display.setCursor(0,  40); display.print("Gyro T "); 
    display.setCursor(50,  40); display.print(temperature, 1); display.print(" C");
    display.display();
    
    count = millis();
    }
    }
    else {
      
    // Serial print and/or display at 0.5 s rate independent of data rates
    delt_t = millis() - count;
    if (delt_t > 500) { // update LCD once per half-second independent of read rate
    
    MPL3115A2ActiveAltimeterMode(); 
    MPL3115A2readAltitude();  // Read the altitude

    MPL3115A2ActiveBarometerMode(); 
    MPL3115A2readPressure();  // Read the pressure
    
    const int station_elevation_m = 1050.0*0.3048; // Accurate for the roof on my house; convert from feet to meters

    float baroin = pressure/100; // pressure is now in millibars

    // Formula to correct absolute pressure in millbars to "altimeter pressure" in inches of mercury 
    // comparable to weather report pressure
    float part1 = baroin - 0.3; //Part 1 of formula
    const float part2 = 0.0000842288;
    float part3 = pow(part1, 0.190284);
    float part4 = (float)station_elevation_m / part3;
    float part5 = (1.0 + (part2 * part4));
    float part6 = pow(part5, 5.2553026);
    float altimeter_setting_pressure_mb = part1 * part6; // Output is now in adjusted millibars
    baroin = altimeter_setting_pressure_mb * 0.02953;
    
    Serial.print("pressure is "); Serial.print(pressure, 2); Serial.println(" Pa");  // Print pressure in Pa
    Serial.print("altitude is "); Serial.print(altitude, 2);  Serial.print(" m, "); // Print altitude in meters   
    Serial.print(altitude/0.3048, 2);  Serial.println(" ft"); // Print altitude in feet 
    Serial.print("temperature is "); Serial.print(temperature, 2);  Serial.print(" C, "); // Print temperature in C
    Serial.print((9.*temperature/5. + 32.), 1);  Serial.println(" F"); // Print temperature in C
    
    display.clearDisplay();   // clears the screen and buffer
    display.setCursor(10, 0); display.print("MPL3115A2");
    display.setCursor(0,10); display.print("P   "); display.setCursor(30,10); display.print(pressure/1000., 2);    display.print(" kPa");
    display.setCursor(0,20); display.print("Alt "); display.setCursor(24,20); display.print(altitude, 1);    display.print("    m");
    display.setCursor(0,30); display.print("Alt "); display.setCursor(24,30); display.print(3.28084*altitude, 1);  display.print("   ft");
    display.setCursor(0,40); display.print("T   "); display.setCursor(30,40); display.print(temperature, 1); display.print("    C");
    display.display();
    
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
    
    tempCount = readTempData();  // Read the adc values
    Temperature = ((float) tempCount) / 333.87 + 21.0; // Temperature in degrees Centigrade
   // Print temperature in degrees Centigrade      
    Serial.print("Temperature is ");  Serial.print(Temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C

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
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI; 
    yaw   -= 13.8; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    roll  *= 180.0f / PI;
     
    if(SerialDebug) {
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(yaw, 2);
    Serial.print(", ");
    Serial.print(pitch, 2);
    Serial.print(", ");
    Serial.println(roll, 2);
    
    Serial.print("rate = "); Serial.print((float)sumCount/sum, 2); Serial.println(" Hz");
    }
   
    display.clearDisplay();    
 
    display.setCursor(0, 0); display.print(" x   y   z  ");

    display.setCursor(0,  8); display.print((int)(1000*ax)); 
    display.setCursor(24, 8); display.print((int)(1000*ay)); 
    display.setCursor(48, 8); display.print((int)(1000*az)); 
    display.setCursor(72, 8); display.print("mg");
    
    display.setCursor(0,  16); display.print((int)(gx)); 
    display.setCursor(24, 16); display.print((int)(gy)); 
    display.setCursor(48, 16); display.print((int)(gz)); 
    display.setCursor(66, 16); display.print("o/s");    

    display.setCursor(0,  24); display.print((int)(mx)); 
    display.setCursor(24, 24); display.print((int)(my)); 
    display.setCursor(48, 24); display.print((int)(mz)); 
    display.setCursor(72, 24); display.print("mG");    
 
    display.setCursor(0,  32); display.print((int)(yaw)); 
    display.setCursor(24, 32); display.print((int)(pitch)); 
    display.setCursor(48, 32); display.print((int)(roll)); 
    display.setCursor(66, 32); display.print("ypr");  
  
    // With these settings the filter is updating at a ~145 Hz rate using the Madgwick scheme and 
    // >200 Hz using the Mahony scheme even though the display refreshes at only 2 Hz.
    // The filter update rate is determined mostly by the mathematical steps in the respective algorithms, 
    // the processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer ODR:
    // an ODR of 10 Hz for the magnetometer produce the above rates, maximum magnetometer ODR of 100 Hz produces
    // filter update rates of 36 - 145 and ~38 Hz for the Madgwick and Mahony schemes, respectively. 
    // This is presumably because the magnetometer read takes longer than the gyro or accelerometer reads.
    // This filter update rate should be fast enough to maintain accurate platform orientation for 
    // stabilization control of a fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
    // produced by the on-board Digital Motion Processor of Invensense's MPU6050 6 DoF and MPU9150 9DoF sensors.
    // The 3.3 V 8 MHz Pro Mini is doing pretty well!
    display.setCursor(0, 40); display.print("rt: "); display.print((float) sumCount / sum, 2); display.print(" Hz"); 
    display.display();

    digitalWrite(myLed, !digitalRead(myLed));
    count = millis(); 
    sumCount = 0;
    sum = 0;    
    }
    }

}

//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

void getMres() {
  switch (Mscale)
  {
 	// Possible magnetometer scales (and their register bit settings) are:
	// 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
          mRes = 10.*4912./8190.; // Proper scale to return milliGauss
          break;
    case MFS_16BITS:
          mRes = 10.*4912./32760.0; // Proper scale to return milliGauss
          break;
  }
}

void getGres() {
  switch (Gscale)
  {
 	// Possible gyro scales (and their register bit settings) are:
	// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
          gRes = 250.0/32768.0;
          break;
    case GFS_500DPS:
          gRes = 500.0/32768.0;
          break;
    case GFS_1000DPS:
          gRes = 1000.0/32768.0;
          break;
    case GFS_2000DPS:
          gRes = 2000.0/32768.0;
          break;
  }
}

void getAres() {
  switch (Ascale)
  {
 	// Possible accelerometer scales (and their register bit settings) are:
	// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
          aRes = 2.0/32768.0;
          break;
    case AFS_4G:
          aRes = 4.0/32768.0;
          break;
    case AFS_8G:
          aRes = 8.0/32768.0;
          break;
    case AFS_16G:
          aRes = 16.0/32768.0;
          break;
  }
}


void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}


void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
}

void readMagData(int16_t * destination)
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

int16_t readTempData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array 
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
  writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
  delay(10);
}


void initMPU9250()
{  
 // wake up device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
  delay(100); // Wait for all registers to reset 

 // get stable time source
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  delay(200); 
  
 // Configure Gyro and Thermometer
 // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
 // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
 // be higher than 1 / 0.0059 = 170 Hz
 // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
 // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  writeByte(MPU9250_ADDRESS, CONFIG, 0x03);  

 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate 
                                    // determined inset in CONFIG above
 
 // Set gyroscope full scale range
 // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5] 
  c = c & ~0x03; // Clear Fchoice bits [1:0] 
  c = c & ~0x18; // Clear GFS bits [4:3]
  c = c | Gscale << 3; // Set full scale range for the gyro
 // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register
  
 // Set accelerometer full-scale range configuration
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5] 
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | Ascale << 3; // Set full scale range for the accelerometer 
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

 // Set accelerometer sample rate configuration
 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

 // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
 // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
  // can join the I2C bus and all can be controlled by the Arduino as master
   writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);    
   writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
   delay(100);
}


// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateMPU9250(float * dest1, float * dest2)
{  
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  
 // reset device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);
   
 // get stable time source; Auto select clock source to be PLL gyroscope reference if ready 
 // else use the internal oscillator, bits 2:0 = 001
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  
  writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
  delay(200);                                    

// Configure device for bias calculation
  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);
  
// Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte(MPU9250_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
 
  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO  
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

// At end of sample accumulation, turn off FIFO sensor read
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
  
  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
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
  writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);
  
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
  readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
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
/*
writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
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

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
   uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
   uint8_t selfTest[6];
   int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
   float factoryTrim[6];
   uint8_t FS = 0;
   
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
  writeByte(MPU9250_ADDRESS, CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, FS<<3);  // Set full scale range for the gyro to 250 dps
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, FS<<3); // Set full scale range for the accelerometer to 2 g

  for( int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer
  
  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
  aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  
    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
  gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  }
  
  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
  aAvg[ii] /= 200;
  gAvg[ii] /= 200;
  }
  
// Configure the accelerometer for self-test
   writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
   writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
   delay(25);  // Delay a while to let the device stabilize

  for( int ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer
  
  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  
    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  }
  
  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
  aSTAvg[ii] /= 200;
  gSTAvg[ii] /= 200;
  }   
  
 // Configure the gyro and accelerometer for normal operation
   writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);  
   writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0x00);  
   delay(25);  // Delay a while to let the device stabilize
   
   // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
   selfTest[0] = readByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL); // X-axis accel self-test results
   selfTest[1] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
   selfTest[2] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
   selfTest[3] = readByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO);  // X-axis gyro self-test results
   selfTest[4] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
   selfTest[5] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

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


void MPL3115A2readAltitude() // Get altitude in meters and temperature in centigrade
{
  uint8_t rawData[5];  // msb/csb/lsb pressure and msb/lsb temperature stored in five contiguous registers 

// We can read the data either by polling or interrupt; see data sheet for relative advantages
// First we try hardware interrupt, which should take less power, etc.
// while (digitalRead(INTP1) == LOW); // Wait for interrupt pin INTP1 to go HIGH
// digitalWrite(INTP1, LOW);  // Reset interrupt pin INTP1
// while((readByte(MPL3115A2_ADDRESS, MPL3115A2_INT_SOURCE) & 0x80) == 0); // Check that the interrupt source is a data ready interrupt
// or use a polling method
// Check data read status; if PTDR (bit 4) not set, then
// toggle OST bit to cause sensor to immediately take a reading
// Setting the one shot toggle is the way to get faster than 1 Hz data read rates
 while ((readByte(MPL3115A2_ADDRESS, MPL3115A2_STATUS) & 0x08) == 0);  MPL3115A2toggleOneShot(); 
  
  readBytes(MPL3115A2_ADDRESS, MPL3115A2_OUT_P_MSB, 5, &rawData[0]);  // Read the five raw data registers into data array

// Altutude bytes-whole altitude contained defined by msb, csb, and first two bits of lsb, fraction by next two bits of lsb
  uint8_t msbA = rawData[0];
  uint8_t csbA = rawData[1];
  uint8_t lsbA = rawData[2];
// Temperature bytes
  uint8_t msbT = rawData[3];
  uint8_t lsbT = rawData[4];
 
 // Calculate altitude, check for negative sign in altimeter data
 long foo = 0;
 if(msbA > 0x7F) {
   foo = ~((long)msbA << 16 | (long)csbA << 8 | (long)lsbA) + 1; // 2's complement the data
   altitude = (float) (foo >> 8) + (float) ((lsbA >> 4)/16.0); // Whole number plus fraction altitude in meters for negative altitude
   altitude *= -1.;
 }
 else {
   altitude = (float) ( (msbA << 8) | csbA) + (float) ((lsbA >> 4)/16.0);  // Whole number plus fraction altitude in meters
 }

// Calculate temperature, check for negative sign
if(msbT > 0x7F) {
 foo = ~(msbT << 8 | lsbT) + 1 ; // 2's complement
 temperature = (float) (foo >> 8) + (float)((lsbT >> 4)/16.0); // add whole and fractional degrees Centigrade
 temperature *= -1.;
 }
 else {
   temperature = (float) (msbT) + (float)((lsbT >> 4)/16.0); // add whole and fractional degrees Centigrade
 }
}

void MPL3115A2readPressure()
{
  uint8_t  rawData[5];  // msb/csb/lsb pressure and msb/lsb temperature stored in five contiguous registers

// We can read the data either by polling or interrupt; see data sheet for relative advantages
// First we try hardware interrupt, which should take less power, etc.
// while (digitalRead(int1Pin) == LOW); // Wait for interrupt pin int1Pin to go HIGH
// digitalWrite(int1Pin, LOW);  // Reset interrupt pin int1Pin
// while((readByte(MPL3115A2_ADDRESS, MPL3115A2_INT_SOURCE) & 0x80) == 0); // Check that the interrupt source is a data ready interrupt
// or use a polling method
// Check data read status; if PTDR (bit 4) not set, then
// toggle OST bit to cause sensor to immediately take a reading
// Setting the one shot toggle is the way to get faster than 1 Hz data read rates
 while ((readByte(MPL3115A2_ADDRESS, MPL3115A2_STATUS) & 0x08) == 0);  MPL3115A2toggleOneShot(); 
 
  readBytes(MPL3115A2_ADDRESS, MPL3115A2_OUT_P_MSB, 5, &rawData[0]);  // Read the five raw data registers into data array

// Pressure bytes
  uint8_t msbP = rawData[0];
  uint8_t csbP = rawData[1];
  uint8_t lsbP = rawData[2];
// Temperature bytes
  uint8_t msbT = rawData[3];
  uint8_t lsbT = rawData[4]; 
 
  long pressure_whole =   ((long)msbP << 16 |  (long)csbP << 8 |  (long)lsbP) ; // Construct whole number pressure
  pressure_whole >>= 6; // Only two most significant bits of lsbP contribute to whole pressure; its an 18-bit number
 
  lsbP &= 0x30; // Keep only bits 5 and 6, the fractional pressure
  lsbP >>= 4; // Shift to get the fractional pressure in terms of quarters of a Pascal
  float pressure_frac = (float) lsbP/4.0; // Convert numbers of fractional quarters to fractional pressure n Pasacl

  pressure = (float) (pressure_whole) + pressure_frac; // Combine whole and fractional parts to get entire pressure in Pascal

// Calculate temperature, check for negative sign
long foo = 0;
if(msbT > 0x7F) { // Is the most significant bit a 1? Then its a negative number in two's complement form
 foo = ~((long) msbT << 8 | lsbT) + 1 ; // 2's complement
 temperature = (float) ((foo >> 8) + ((lsbT >> 4)/16.0)); // add whole and fractional degrees Centigrade
 temperature *= -1.;
 }
 else {
   temperature = (float) (msbT) + (float)((lsbT >> 4)/16.0); // add whole and fractional degrees Centigrade
 }
}

/*
=====================================================================================================
Define functions according to 
"Data Manipulation and Basic Settings of the MPL3115A2 Command Line Interface Drive Code"
by Miguel Salhuana
Freescale Semiconductor Application Note AN4519 Rev 0.1, 08/2012
=====================================================================================================
*/
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Clears then sets OST bit which causes the sensor to immediately take another reading
void MPL3115A2toggleOneShot()
{
    MPL3115A2Active();  // Set to active to start reading
    uint8_t c = readByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1);
    writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, c & ~(1<<1)); // Clear OST (bit 1)
    c = readByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1);
    writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, c | (1<<1)); // Set OST bit to 1
}
    
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Set the Outputting Sample Rate
void MPL3115A2SampleRate(uint8_t samplerate)
{
  MPL3115A2Standby();  // Must be in standby to change registers

  uint8_t c = readByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1);
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, c & ~(0x38)); // Clear OSR bits 3,4,5
  if(samplerate < 8) { // OSR between 1 and 7
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, c | (samplerate << 3));  // Write OSR to bits 3,4,5
  }
  
  MPL3115A2Active();  // Set to active to start reading
 }
 
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Initialize the MPL3115A2 registers for FIFO mode
void initFIFOMPL3115A2()
{
  // Clear all interrupts by reading the data output registers
  uint8_t temp;
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_OUT_P_MSB);
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_OUT_P_CSB);
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_OUT_P_LSB);
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_OUT_T_MSB);
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_OUT_T_LSB);
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_F_STATUS);
  
   MPL3115A2Standby();  // Must be in standby to change registers
  
  // Set CTRL_REG4 register to configure interupt enable
  // Enable data ready interrupt (bit 7), enable FIFO (bit 6), enable pressure window (bit 5), temperature window (bit 4),
  // pressure threshold (bit 3), temperature threshold (bit 2), pressure change (bit 1) and temperature change (bit 0)
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG4, 0x40);  // enable FIFO
  
  //  Configure INT 1 for data ready, all other (inc. FIFO) interrupts to INT2
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG5, 0x80); 
  
  // Set CTRL_REG3 register to configure interupt signal type
  // Active HIGH, push-pull interupts INT1 and INT 2
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG3, 0x22); 
  
  // Set FIFO mode
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_F_SETUP, 0x00); // Clear FIFO mode
// In overflow mode, when FIFO fills up, no more data is taken until the FIFO registers are read
// In watermark mode, the oldest data is overwritten by new data until the FIFO registers are read
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_F_SETUP, 0x80); // Set F_MODE to interrupt when overflow = 32 reached
//  writeByte(MPL3115A2_ADDRESS, MPL3115A2_F_SETUP, 0x60); // Set F_MODE to accept 32 data samples and interrupt when watermark = 32 reached

  MPL3115A2Active();  // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Initialize the MPL3115A2 for realtime data collection 
void initRealTimeMPL3115A2()
{
  // Clear all interrupts by reading the data output registers
  uint8_t temp;
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_OUT_P_MSB);
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_OUT_P_CSB);
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_OUT_P_LSB);
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_OUT_T_MSB);
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_OUT_T_LSB);
  temp = readByte(MPL3115A2_ADDRESS, MPL3115A2_F_STATUS);
  
   MPL3115A2Standby();  // Must be in standby to change registers
  
  // Set CTRL_REG4 register to configure interupt enable
  // Enable data ready interrupt (bit 7), enable FIFO (bit 6), enable pressure window (bit 5), temperature window (bit 4),
  // pressure threshold (bit 3), temperature threshold (bit 2), pressure change (bit 1) and temperature change (bit 0)
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG4, 0x80);  
  
  //  Configure INT 1 for data ready, all other interrupts to INT2
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG5, 0x80); 
  
  // Set CTRL_REG3 register to configure interupt signal type
  // Active HIGH, push-pull interupts INT1 and INT 2
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG3, 0x22); 
  
  // Set FIFO mode
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_F_SETUP, 0x00); // disable FIFO mode
  
  MPL3115A2Active();  // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Set the Auto Acquisition Time Step
void MPL3115A2TimeStep(uint8_t ST_Value)
{
 MPL3115A2Standby(); // First put device in standby mode to allow write to registers
 
 uint8_t c = readByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG2); // Read contents of register CTRL_REG2
 if (ST_Value <= 0xF) {
 writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG2, (c | ST_Value)); // Set time step n from 0x0 to 0xF (bits 0 - 3) for time intervals from 1 to 32768 (2^n) seconds
 }
 
 MPL3115A2Active(); // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Enable the pressure and temperature event flags
 // Bit 2 is general data ready event mode on new Pressure/Altitude or temperature data
 // Bit 1 is event flag on new Pressure/Altitude data
 // Bit 0 is event flag on new Temperature data
void MPL3115A2enableEventflags()
{
  MPL3115A2Standby();  // Must be in standby to change registers
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_PT_DATA_CFG, 0x07); //Enable all three pressure and temperature event flags
  MPL3115A2Active();  // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Enter Active Altimeter mode
void MPL3115A2ActiveAltimeterMode()
{
 MPL3115A2Standby(); // First put device in standby mode to allow write to registers
 uint8_t c = readByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1); // Read contents of register CTRL_REG1
 writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, c | (0x80)); // Set ALT (bit 7) to 1
 MPL3115A2Active(); // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Enter Active Barometer mode
void MPL3115A2ActiveBarometerMode()
{
 MPL3115A2Standby(); // First put device in standby mode to allow write to registers
 uint8_t c = readByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1); // Read contents of register CTRL_REG1
 writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, c & ~(0x80)); // Set ALT (bit 7) to 0
 MPL3115A2Active(); // Set to active to start reading
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Software resets the MPL3115A2.
// It must be in standby to change most register settings
void MPL3115A2Reset()
{
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, 0x04); // Set RST (bit 2) to 1
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Sets the MPL3115A2 to standby mode.
// It must be in standby to change most register settings
void MPL3115A2Standby()
{
  uint8_t c = readByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1); // Read contents of register CTRL_REG1
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, c & ~(0x01)); // Set SBYB (bit 0) to 0
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Sets the MPL3115A2 to active mode.
// Needs to be in this mode to output data
void MPL3115A2Active()
{
  uint8_t c = readByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1); // Read contents of register CTRL_REG1
  writeByte(MPL3115A2_ADDRESS, MPL3115A2_CTRL_REG1, c | 0x01); // Set SBYB (bit 0) to 1
}       

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
	Wire.requestFrom(address, (size_t) 1);  // Read one byte from slave register address 
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


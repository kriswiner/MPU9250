#include <SPI.h>

const int CS_PIN = 2;

#define READ_FLAG 0x80
#define SPI_DATA_RATE 100000 // 1 MHz max
#define SPI_MODE SPI_MODE3

#define SERIAL_DEBUG true

void setup()
{
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  SPI.begin();
  Serial.begin(38400);

  kickHardware();

  ak8963Init();
  //resetAndWake();

  Serial.println("Calling ak8963WhoAmI()");
  ak8963WhoAmI();

  checkSPI();
} // end setup()

void loop() {
}

// Read the WHOAMI (WIA) register of the AK8963
uint8_t ak8963WhoAmI()
{
  uint8_t response;
  // Set the I2C slave addres of AK8963 and set for read
  response = writeByteSPI(0x25, 0x0C|READ_FLAG);
  // I2C slave 0 register address from where to begin data transfer
  response = writeByteSPI(0x26, 0x00);
  // Enable 1-byte reads on slave 0
  response = writeByteSPI(0x27, 0x81);
  delayMicroseconds(1);
  // Read WIA register
  response = writeByteSPI(0x49|READ_FLAG, 0x00);

  return response;
}

// Initialize the AK8963
void ak8963Init()
{
  uint8_t ret;

  // Reset registers to defaults, bit auto clears
  ret = writeByteSPI(0x6B, 0x80);
#ifdef SERIAL_DEBUG
  Serial.print("write(0x6B, 0x80): 0x");
  Serial.println(ret, HEX);
#endif

  // Auto select the best available clock source
  ret = writeByteSPI(0x6B, 0x01);
#ifdef SERIAL_DEBUG
  Serial.print("write(0x6B, 0x01): 0x");
  Serial.println(ret, HEX);
#endif

  // Enable X,Y, & Z axes of accel and gyro
  ret = writeByteSPI(0x6C, 0x00);
#ifdef SERIAL_DEBUG
  Serial.print("write(0x6C, 0x00): 0x");
  Serial.println(ret, HEX);
#endif

  // Config disable FSYNC pin, set gyro/temp bandwidth to 184/188 Hz
  ret = writeByteSPI(0x1A, 0x01);
#ifdef SERIAL_DEBUG
  Serial.print("write(0x1A, 0x01): 0x");
  Serial.println(ret, HEX);
#endif

  // Self tests off, gyro set to +/-2000 dps FS
  ret = writeByteSPI(0x1B, 0x18);
#ifdef SERIAL_DEBUG
  Serial.print("write(0x1B, 0x18): 0x");
  Serial.println(ret, HEX);
#endif

  // Self test off, accel set to +/- 8g FS
  ret = writeByteSPI(0x1C, 0x08);
#ifdef SERIAL_DEBUG
  Serial.print("write(0x1C, 0x08): 0x");
  Serial.println(ret, HEX);
#endif

  // Bypass DLPF and set accel bandwidth to 184 Hz
  ret = writeByteSPI(0x1D, 0x09);
#ifdef SERIAL_DEBUG
  Serial.print("write(0x1D, 0x09): 0x");
  Serial.println(ret, HEX);
#endif

  // Configure INT pin (active high / push-pull / latch until read)
  ret = writeByteSPI(0x37, 0x30);
#ifdef SERIAL_DEBUG
  Serial.print("write(0x37, 0x30): 0x");
  Serial.println(ret, HEX);
#endif

  // Enable I2C master mode
  // TODO Why not do this 11-100 ms after power up?
  ret = writeByteSPI(0x6A, 0x20);
#ifdef SERIAL_DEBUG
  Serial.print("write(0x6A, 0x20): 0x");
  Serial.println(ret, HEX);
#endif

  // Disable multi-master and set I2C master clock to 400 kHz
  //https://developer.mbed.org/users/kylongmu/code/MPU9250_SPI/ calls says
  // enabled multi-master... TODO Find out why
  ret = writeByteSPI(0x24, 0x0D);
#ifdef SERIAL_DEBUG
  Serial.print("write(0x24, 0x0D): 0x");
  Serial.println(ret, HEX);
#endif

  // Set to write to slave address 0x0C
  // TODO: Verify this address
  ret = writeByteSPI(0x25, 0x0C);
#ifdef SERIAL_DEBUG
  Serial.print("write(0x25, 0x0C): 0x");
  Serial.println(ret, HEX);
#endif


  // Point save 0 register at AK8963's control 2 (soft reset) register
  ret = writeByteSPI(0x26, 0x0B);
#ifdef SERIAL_DEBUG
  Serial.print("write(0x26, 0x0B): 0x");
  Serial.println(ret, HEX);
#endif

  // Send 0x01 to AK8963 via slave 0 to trigger a soft restart
  ret = writeByteSPI(0x63, 0x01);
#ifdef SERIAL_DEBUG
  Serial.print("write(0x63, 0x01): 0x");
  Serial.println(ret, HEX);
#endif

  // Enable simple 1-byte I2C reads from slave 0
  ret = writeByteSPI(0x27, 0x81);
#ifdef SERIAL_DEBUG
  Serial.print("write(0x27, 0x81): 0x");
  Serial.println(ret, HEX);
#endif

  // Point save 0 register at AK8963's control 1 (mode) register
  ret = writeByteSPI(0x26, 0x0A);
#ifdef SERIAL_DEBUG
  Serial.print("write(0x26, 0x0A): 0x");
  Serial.println(ret, HEX);
#endif

  // 16-bit continuous measurement mode 1
  ret = writeByteSPI(0x63, 0x12);
#ifdef SERIAL_DEBUG
  Serial.print("write(0x63, 0x12): 0x");
  Serial.println(ret, HEX);
#endif

  // Enable simple 1-byte I2C reads from slave 0
  ret = writeByteSPI(0x27, 0x81);
#ifdef SERIAL_DEBUG
  Serial.print("write(0x27, 0x81): 0x");
  Serial.println(ret, HEX);
#endif
}

void checkSPI()
{
  uint8_t temp = SPSR;
  if(temp)
  {
    Serial.print("SPSR – SPI Status Register: 0b");
    Serial.println(temp);
  }
}


// Reset and wake the MPU-9250
void resetAndWake()
{
#ifdef SERIAL_DEBUG
  Serial.println("Resetting and waking MPU-9250");
#endif
#ifdef SERIAL_DEBUG
  Serial.println("Resetting (0x80 => 0x6B)");
#endif
  writeByteSPI(0x80, 0x6B); // Reset
  delay(100);
#ifdef SERIAL_DEBUG
  Serial.println("Waking (0x00 => 0x6B)");
#endif
  writeByteSPI(0x00, 0x6B); // Wake
}


// Write a null byte w/o CS assertion to get SPI hardware to idle high (mode 3)
void kickHardware()
{
  SPI.beginTransaction(SPISettings(SPI_DATA_RATE, MSBFIRST, SPI_MODE));
  SPI.transfer(0x00); // Send null byte
  SPI.endTransaction();
}


// Function to write a byte on SPI bus.
uint8_t writeByteSPI(uint8_t registerAddress, uint8_t writeData)
{
  digitalWrite(CS_PIN, LOW);
  SPI.beginTransaction(SPISettings(SPI_DATA_RATE, MSBFIRST, SPI_MODE));
  SPI.transfer(registerAddress);
  uint8_t returnVal = SPI.transfer(writeData);
  SPI.endTransaction();
  digitalWrite(CS_PIN, HIGH);
#ifdef SERIAL_DEBUG
  Serial.print("BHW::writeByteSPI slave returned: 0x");
  Serial.println(returnVal, HEX);
#endif
  return returnVal;
}


// Function to read a byte from SPI bus.
uint8_t readByteSPI(uint8_t registerAddress)
{
  return writeByteSPI(registerAddress | READ_FLAG, 0xFF /*0xFF is arbitrary*/);
}

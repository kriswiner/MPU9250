/* SPIFlash.cpp
Sketch by Kris Winer December 16. 2016

License: Use this sketch any way you choose; if you like it, buy me a beer sometime

Purpose: Checks function of a variety of SPI NOR flash memory chips hosted by the STM32L4
Dragonfly (STM32L476), Butterfly (STM32L433), and Ladybug (STML432) development boards or their variants.

Sketch takes advantage of the SPI.beginTransaction/SPI.EndTransaction protocol for efficiency
and maximum speed.

Sketch based on the work of Pete (El Supremo) as follows:
 * Copyright (c) 2014, Pete (El Supremo), el_supremo@shaw.ca
 *
 * Development of this audio library was funded by PJRC.COM, LLC by sales of
 * Teensy and Audio Adaptor boards.  Please support PJRC's efforts to develop
 * open source software by purchasing Teensy or other PJRC products.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 */

#include "SPIFlash.h"
#include "SPI.h"

SPIFlash::SPIFlash(uint8_t CSPIN)
{
  _csPin = CSPIN; 
}


void SPIFlash::SPIFlashinit()
{
  SPI.begin();
}


void SPIFlash::getChipID()
{ 
  Serial.print("ID bytes: ");
  uint16_t id[3];
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);
  SPI.transfer(0x9F);
  id[0] = SPI.transfer(0);
  id[1] = SPI.transfer(0);
  id[2] = SPI.transfer(0);
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();
  Serial.print(id[0], HEX); Serial.print(" "); Serial.print(id[1], HEX);  Serial.print(" ");  Serial.println(id[2], HEX); 

  Serial.println("Winbond  W25Q80BLUX1G   Chip ID = 0xEF, 0x40, 0x14, 0x0");
  Serial.println("Macronix MX25L12835FZNI Chip ID = 0xC2, 0x20, 0x18, 0xC2");
  Serial.println("Spansion S25FL127S      Chip ID = 0x01, 0x20, 0x18, 0x4D");
  Serial.println(" ");
}


void SPIFlash::write_pause(void)
{
  if(flash_wait_for_write) {
    while(flash_read_status() & STAT_WIP);
    flash_wait_for_write = 0;
  }
}

//=====================================
// convert a page number to a 24-bit address
int SPIFlash::page_to_address(int pn)
{
  return(pn << 8);
}

//=====================================
// convert a 24-bit address to a page number
int SPIFlash::address_to_page(int addr)
{
  return(addr >> 8);
}

//=====================================
void SPIFlash::flash_read_id(unsigned char *idt)
{
  write_pause();
  //set control register 
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);
  SPI.transfer(CMD_READ_ID);
  for(uint16_t i = 0; i < 20; i++) {
    *idt++ = SPI.transfer(0x00);
  }
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();
}

//=====================================
unsigned char SPIFlash::flash_read_status(void)
{
  unsigned char c;

// This can't do a write_pause
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);  
  SPI.transfer(CMD_READ_STATUS_REG);
  c = SPI.transfer(0x00);
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();
  return(c);
}

//=====================================

void SPIFlash::flash_hard_reset(void)
{
  // Make sure that the device is not busy before
  // doing the hard reset sequence
  // At the moment this does NOT check the
  // SUSpend status bit in Status Register 2
  // but the library does not support suspend
  // mode yet anyway
  write_pause();
  
  // Send Write Enable command
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);
  SPI.transfer(CMD_RESET_DEVICE );
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();
  delayMicroseconds(50);
  // Wait for the hard reset to finish
  // Don't use flash_wait_for_write here
  while(flash_read_status() & STAT_WIP);
  // The spec says "the device will take
  // approximately tRST=30 microseconds
  // to reset"
}

//=====================================
void SPIFlash::flash_chip_erase(boolean wait)
{
  write_pause();
  // Send Write Enable command
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);
  SPI.transfer(CMD_WRITE_ENABLE);
  digitalWrite(_csPin, HIGH);
  digitalWrite(_csPin, LOW);
  SPI.transfer(CMD_CHIP_ERASE);
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();
  flash_wait_for_write = 1;
  if(wait)write_pause();
}

//=====================================
// Tse Typ=0.6sec Max=3sec
// measured 549.024ms
// Erase the sector which contains the specified
// page number.
// The smallest unit of memory which can be erased
// is the 4kB sector (which is 16 pages)
void SPIFlash::flash_erase_pages_sector(int pn)
{
  int address;

  write_pause(); 
  // Send Write Enable command
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);
  SPI.transfer(CMD_WRITE_ENABLE);
  digitalWrite(_csPin, HIGH);
  
  digitalWrite(_csPin, LOW);
  SPI.transfer(CMD_SECTOR_ERASE);
  // Send the 3 byte address
  address = page_to_address(pn);
  SPI.transfer((address >> 16) & 0xff);
  SPI.transfer((address >> 8) & 0xff);
  SPI.transfer(address & 0xff);
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();  
  // Indicate that next I/O must wait for this write to finish
  flash_wait_for_write = 1;
}

//=====================================
// Erase the 32kb block which contains the specified
// page number.
void SPIFlash::flash_erase_pages_block32k(int pn)
{
  int address;

  write_pause();
  // Send Write Enable command
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);
  SPI.transfer(CMD_WRITE_ENABLE);
  digitalWrite(_csPin, HIGH);
  digitalWrite(_csPin, LOW);
  SPI.transfer(CMD_BLOCK32K_ERASE);
  // Send the 3 byte address
  address = page_to_address(pn);
  SPI.transfer((address >> 16) & 0xFF);
  SPI.transfer((address >> 8) & 0xFF);
  SPI.transfer(address & 0xFF);
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();
  // Indicate that next I/O must wait for this write to finish
  flash_wait_for_write = 1;
}

//=====================================
// Erase the 64kb block which contains the specified
// page number.
void SPIFlash::flash_erase_pages_block64k(int pn)
{
  int address;
  
  write_pause();
  // Send Write Enable command
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);
  SPI.transfer(CMD_WRITE_ENABLE);
  digitalWrite(_csPin, HIGH);
  digitalWrite(_csPin, LOW);
  SPI.transfer(CMD_BLOCK64K_ERASE);
  // Send the 3 byte address
  address = page_to_address(pn);
  SPI.transfer((address >> 16) & 0xFF);
  SPI.transfer((address >> 8) & 0xFF);
  SPI.transfer(address & 0xFF);
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();
  // Indicate that next I/O must wait for this write to finish
  flash_wait_for_write = 1;
}

//=====================================
void SPIFlash::flash_page_program(unsigned char *wp,int pn)
{
  int address;

  write_pause(); 
  // Send Write Enable command
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);
  SPI.transfer(CMD_WRITE_ENABLE);
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();
  
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);
  SPI.transfer(CMD_PAGE_PROGRAM);
  // Send the 3 byte address
  address = page_to_address(pn);
  SPI.transfer((address >> 16) & 0xFF);
  SPI.transfer((address >> 8) & 0xFF);
  SPI.transfer(address & 0xFF);
  // Now write 256 bytes to the page
  for(uint16_t i = 0; i < 256; i++) {
  SPI.transfer(*wp++);
  }
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();
  // Indicate that next I/O must wait for this write to finish
  flash_wait_for_write = 1;
}

//=====================================
void SPIFlash::flash_read_pages(unsigned char *p,int pn,const int n_pages)
{
  int address;
  unsigned char *rp = p;
  
  write_pause();
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);
  SPI.transfer(CMD_READ_DATA);
  // Send the 3 byte address
  address = page_to_address(pn);
  SPI.transfer((address >> 16) & 0xFF);
  SPI.transfer((address >> 8) & 0xFF);
  SPI.transfer(address & 0xFF);
  // Now read the page's data bytes
  for(uint16_t i = 0; i < n_pages * 256; i++) {
    *rp++ = SPI.transfer(0);
  }
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();
}

//=====================================
// Read specified number of pages starting with pn
void SPIFlash::flash_fast_read_pages(unsigned char *p,int pn,const int n_pages)
{
  int address;
  unsigned char *rp = p;
  
  write_pause();
// The chip doesn't run at the higher clock speed until
// after the command and address have been sent
  SPI.beginTransaction(SPISettings(50000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);
  SPI.transfer(CMD_READ_HIGH_SPEED);
  // Send the 3 byte address
  address = page_to_address(pn);
  SPI.transfer((address >> 16) & 0xFF);
  SPI.transfer((address >> 8) & 0xFF);
  SPI.transfer(address & 0xFF);
  // send dummy byte
  SPI.transfer(0);
  // Now read the number of pages required
  for(uint16_t i = 0; i < n_pages * 256; i++) {
    *rp++ = SPI.transfer(0);
  }
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();
}


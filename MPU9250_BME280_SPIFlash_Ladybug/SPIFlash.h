/* SPIFlash.h
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

#ifndef SPIFlash_h
#define SPIFlash_h

#include "Arduino.h"
#include "SPI.h"

#define STAT_WIP 1
#define STAT_WEL 2

#define CMD_WRITE_STATUS_REG   0x01
#define CMD_PAGE_PROGRAM       0x02
#define CMD_READ_DATA          0x03
#define CMD_WRITE_DISABLE      0x04 
#define CMD_READ_STATUS_REG    0x05
#define CMD_WRITE_ENABLE       0x06
#define CMD_READ_HIGH_SPEED    0x0B 
#define CMD_SECTOR_ERASE       0x20 
#define CMD_BLOCK32K_ERASE     0x52 
#define CMD_RESET_DEVICE       0xF0 
#define CMD_READ_ID            0x9F
#define CMD_RELEASE_POWER_DOWN 0xAB 
#define CMD_POWER_DOWN         0xB9 
#define CMD_CHIP_ERASE         0xC7
#define CMD_BLOCK64K_ERASE     0xD8 

class SPIFlash
{
  public: 
  SPIFlash(uint8_t CSPIN);
  void SPIFlashinit();
  void getChipID();
  void write_pause(void);
  int page_to_address(int pn);
  int address_to_page(int addr);
  void flash_read_id(unsigned char *idt);
  unsigned char flash_read_status(void);
  void flash_hard_reset(void);
  void flash_chip_erase(boolean wait);
  void flash_erase_pages_sector(int pn);
  void flash_erase_pages_block32k(int pn);
  void flash_erase_pages_block64k(int pn);
  void flash_page_program(unsigned char *wp,int pn);
  void flash_read_pages(unsigned char *p,int pn,const int n_pages);
  void flash_fast_read_pages(unsigned char *p,int pn,const int n_pages);
  private:
  uint8_t   _csPin;
  unsigned char flash_wait_for_write = 0;
};

#endif

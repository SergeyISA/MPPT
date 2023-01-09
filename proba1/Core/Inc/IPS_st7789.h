/*
 * IPS_st7789.h
 *
 *  Created on: 9 ��� 2019 �.
 *      Author: vvv
 */

#ifndef IPS_ST7789_H_
#define IPS_ST7789_H_

#include "stm32f1xx_hal.h"
#include "IPS_st7789_fonts.h"

#define ST7789_IPS_WIDTH_240 	240
#define ST7789_IPS_HEIGHT_240 	240

#define ST7789_240x240_XSTART 0
#define ST7789_240x240_YSTART 0 //????????????????????????????????

#define ST_CMD_DELAY   0x80    // special signifier for command lists

#define ST7789_NOP     0x00
#define ST7789_SWRESET 0x01
#define ST7789_RDDID   0x04
#define ST7789_RDDST   0x09

#define ST7789_SLPIN   0x10
#define ST7789_SLPOUT  0x11
#define ST7789_PTLON   0x12
#define ST7789_NORON   0x13

#define ST7789_INVOFF  0x20
#define ST7789_INVON   0x21
#define ST7789_DISPOFF 0x28
#define ST7789_DISPON  0x29
#define ST7789_CASET   0x2A
#define ST7789_RASET   0x2B
#define ST7789_RAMWR   0x2C
#define ST7789_RAMRD   0x2E

#define ST7789_PTLAR   0x30
#define ST7789_COLMOD  0x3A
#define ST7789_MADCTL  0x36

#define ST7789_MADCTL_MY  0x80
#define ST7789_MADCTL_MX  0x40
#define ST7789_MADCTL_MV  0x20
#define ST7789_MADCTL_ML  0x10
#define ST7789_MADCTL_RGB 0x00

#define ST7789_RDID1   0xDA
#define ST7789_RDID2   0xDB
#define ST7789_RDID3   0xDC
#define ST7789_RDID4   0xDD

// Color definitions
#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x05E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE 0xFFFF



void IPS_st7789_Init(void);
void IPS_st7789_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void IPS_st7789_WriteChar(uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor);
void IPS_st7789_WriteString(uint16_t x, uint16_t y, const char* str, FontDef font, uint16_t color, uint16_t bgcolor);
void IPS_st7789_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void IPS_st7789_FillScreen(uint16_t color);
void IPS_st7789_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data);
void IPS_st7789_InvertColors(uint8_t invert);
void IPS_st7789_WriteU8_2(uint16_t x, uint16_t y, uint16_t dig, uint16_t zap, FontDef font, uint16_t color, uint16_t bgcolor);
void IPS_st7789_WriteU16(uint16_t x, uint16_t y, uint16_t dig, uint16_t zap, FontDef font, uint16_t color, uint16_t bgcolor);
void IPS_st7789_WriteSIM(uint16_t x, uint16_t y, uint8_t* str,uint8_t len, FontDef font, uint16_t color, uint16_t bgcolor) ;
void IPS_st7789_WriteF(uint16_t x, uint16_t y, float dig, FontDef font, uint16_t color, uint16_t bgcolor);
void Preobr_LCD (uint16_t c);
#endif /* IPS_ST7789_H_ */

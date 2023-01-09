/*
 * IPS_st7789.c
 *
 *  Created on: 9 ��� 2019 �.
 *      Author: vvv
 */

#include "main.h"
#include "stm32f1xx_hal.h"
#include "IPS_st7789.h"

extern SPI_HandleTypeDef hspi1;
#define IPS_st7789_SPI_PORT hspi1

unsigned char edi_LCD, des_LCD, sot_LCD, tis_LCD, dts_LCD, sts_LCD ;
extern uint8_t Stroka, Poz;


// based on Adafruit ST7735 library for Arduino
static const uint8_t

cmd_240x240[] = {                 		// Initialization commands for 7789 screens
   9,                       				// 9 commands in list:
   ST7789_SWRESET,   ST_CMD_DELAY,  		// 1: Software reset, no args, w/delay
     150,                     				// 150 ms delay
   ST7789_SLPOUT ,   ST_CMD_DELAY,  		// 2: Out of sleep mode, no args, w/delay
     255,                    				// 255 = 500 ms delay
   ST7789_COLMOD , 1+ST_CMD_DELAY,  		// 3: Set color mode, 1 arg + delay:
     0x55,                   				// 16-bit color
     10,                     				// 10 ms delay
   ST7789_MADCTL , 1,  					// 4: Memory access ctrl (directions), 1 arg:
     0x00,                   				// Row addr/col addr, bottom to top refresh
   ST7789_CASET  , 4,  					// 5: Column addr set, 4 args, no delay:
     0x00, ST7789_240x240_XSTART,          // XSTART = 0
	 (240+ST7789_240x240_XSTART) >> 8,
	 (240+ST7789_240x240_XSTART) & 0xFF,   // XEND = 240
   ST7789_RASET  , 4,  					// 6: Row addr set, 4 args, no delay:
     0x00, ST7789_240x240_YSTART,          // YSTART = 0
     (240+ST7789_240x240_YSTART) >> 8,
	 (240+ST7789_240x240_YSTART) & 0xFF,	// YEND = 240
	 ST7789_INVON ,   ST_CMD_DELAY,  		// 7: Inversion ON
     10,
   ST7789_NORON  ,   ST_CMD_DELAY,  		// 8: Normal display on, no args, w/delay
     10,                     				// 10 ms delay
   ST7789_DISPON ,   ST_CMD_DELAY,  		// 9: Main screen turn on, no args, w/delay
   	 255
}; // 255 = 500 ms delay


void	Preobr_LCD (uint16_t c)
{
	sts_LCD = (c%1000000UL)/100000UL;
	dts_LCD = (c%100000UL)/10000UL;
	tis_LCD = (c%10000UL)/1000UL;
	sot_LCD = (c%1000UL)/100UL;
	des_LCD = (c%100UL)/10UL;
	edi_LCD =  c%10UL;
	return;
}


static void IPS_st7789_Reset()
{
    HAL_GPIO_WritePin(IPS_Res_GPIO_Port, IPS_Res_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(IPS_Res_GPIO_Port, IPS_Res_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(IPS_Res_GPIO_Port, IPS_Res_Pin, GPIO_PIN_SET);
    HAL_Delay(500);
}

static void IPS_st7789_WriteCommand(uint8_t cmd)
{
    HAL_GPIO_WritePin(IPS_DC_GPIO_Port, IPS_DC_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&IPS_st7789_SPI_PORT, &cmd, sizeof(cmd), HAL_MAX_DELAY);
}

static void IPS_st7789_WriteData(uint8_t* buff, uint8_t buff_size)
{
    HAL_GPIO_WritePin(IPS_DC_GPIO_Port, IPS_DC_Pin, GPIO_PIN_SET);
    HAL_SPI_Transmit(&IPS_st7789_SPI_PORT, buff, buff_size, HAL_MAX_DELAY);
}

static void IPS_st7789_ExecuteCommandList(const uint8_t *addr) {
    uint8_t numCommands, numArgs;
    uint16_t ms;

    numCommands = *addr++;
    while(numCommands--)
    {
        uint8_t cmd = *addr++;
        IPS_st7789_WriteCommand(cmd);

        numArgs = *addr++;

        ms = numArgs & ST_CMD_DELAY; // If high bit set, delay follows args
        numArgs &= ~ST_CMD_DELAY;

        if(numArgs)
        {
        	IPS_st7789_WriteData((uint8_t*)addr, numArgs);
            addr += numArgs;
        }

        if(ms)
        {
            ms = *addr++;
            if(ms == 255) {ms = 500;}
            HAL_Delay(ms);
        }
    }
}



static void IPS_st7789_SetAddressWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
    // column address set
	IPS_st7789_WriteCommand(ST7789_CASET );
    uint8_t data[] = { 0x00, x0 + ST7789_240x240_XSTART , 0x00, x1 + ST7789_240x240_XSTART  };
    IPS_st7789_WriteData(data, sizeof(data));

    // row address set
    IPS_st7789_WriteCommand(ST7789_RASET );
    data[1] = y0 + ST7789_240x240_YSTART ;
    data[3] = y1 + ST7789_240x240_YSTART ;
    IPS_st7789_WriteData(data, sizeof(data));

    // write to RAM
    IPS_st7789_WriteCommand(ST7789_RAMWR );
}

void IPS_st7789_Init()
{
	IPS_st7789_Reset();
	IPS_st7789_ExecuteCommandList(cmd_240x240);
}

void IPS_st7789_DrawPixel(uint16_t x, uint16_t y, uint16_t color) {
    if((x >= ST7789_IPS_WIDTH_240) || (y >= ST7789_IPS_HEIGHT_240))
        return;

    IPS_st7789_SetAddressWindow(x, y, x+1, y+1);
    uint8_t data[] = { color >> 8, color & 0xFF };
    IPS_st7789_WriteData(data, sizeof(data));
}

 void IPS_st7789_WriteChar(uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor) {
    uint32_t i, b, j;

    IPS_st7789_SetAddressWindow(x, y, x+font.width-1, y+font.height-1);

    for(i = 0; i < font.height; i++) {
        b = font.data[(ch - 32) * font.height + i];
        for(j = 0; j < font.width; j++) {
            if((b << j) & 0x8000)  {
                uint8_t data[] = { color >> 8, color & 0xFF };
                IPS_st7789_WriteData(data, sizeof(data));
            } else {
                uint8_t data[] = { bgcolor >> 8, bgcolor & 0xFF };
                IPS_st7789_WriteData(data, sizeof(data));
            }
        }
    }
}

 void IPS_st7789_WriteU16(uint16_t x, uint16_t y, uint16_t dig, uint16_t zap, FontDef font, uint16_t color, uint16_t bgcolor)
 {
	 int flag = 0;


	 Preobr_LCD (dig);

	 IPS_st7789_WriteChar(x                     , y, ' ', font, color, bgcolor);
	 IPS_st7789_WriteChar(x +     font.width - 1, y, ' ', font, color, bgcolor);
	 IPS_st7789_WriteChar(x + 2 * font.width - 1, y, ' ', font, color, bgcolor);
//	 IPS_st7789_WriteChar(x + 3 * font.width - 1, y, ' ', font, color, bgcolor);
//	 IPS_st7789_WriteChar(x + 4 * font.width - 1, y, ' ', font, color, bgcolor);
//	 IPS_st7789_WriteChar(x + 5 * font.width - 1, y, ' ', font, color, bgcolor);

	 if((sts_LCD) || (zap == 5))           {IPS_st7789_WriteChar(x, y, sts_LCD + 0x30, font, color, bgcolor); flag = 1;x +=font.width - 1;}
	 if(zap == 5){IPS_st7789_WriteChar(x, y, ',', font, color, bgcolor); x += font.width - 1;}
	 if((dts_LCD) || (flag) || (zap == 4)) {IPS_st7789_WriteChar(x, y, dts_LCD + 0x30, font, color, bgcolor); flag = 1;x +=font.width - 1;}
	 if(zap == 4){IPS_st7789_WriteChar(x, y, ',', font, color, bgcolor); x += font.width - 1;}
	 if((tis_LCD) || (flag) || (zap == 3)) {IPS_st7789_WriteChar(x, y, tis_LCD + 0x30, font, color, bgcolor); flag = 1;x +=font.width - 1;}
	 if(zap == 3){IPS_st7789_WriteChar(x, y, ',', font, color, bgcolor); x += font.width - 1;}
	 if((sot_LCD) || (flag) || (zap == 2)) {IPS_st7789_WriteChar(x, y, sot_LCD + 0x30, font, color, bgcolor); flag = 1;x +=font.width - 1;}
	 if(zap == 2){IPS_st7789_WriteChar(x, y, ',', font, color, bgcolor); x += font.width - 1;}
	 if((des_LCD) || (flag) || (zap == 1)) {IPS_st7789_WriteChar(x, y, des_LCD + 0x30, font, color, bgcolor); flag = 1;x +=font.width - 1;}
	 if(zap == 1){IPS_st7789_WriteChar(x, y, ',', font, color, bgcolor); x += font.width - 1;}
	                          IPS_st7789_WriteChar(x, y, edi_LCD + 0x30, font, color, bgcolor); flag = 1;
	 Poz = x + font.width - 1;
 }

 void IPS_st7789_WriteU8_2(uint16_t x, uint16_t y, uint16_t dig, uint16_t zap, FontDef font, uint16_t color, uint16_t bgcolor)
  {
 	 int flag = 0;

 	 Preobr_LCD (dig);

 	 IPS_st7789_WriteChar(x                     , y, ' ', font, color, bgcolor);
 	 IPS_st7789_WriteChar(x +     font.width - 1, y, ' ', font, color, bgcolor);


 	 IPS_st7789_WriteChar(x, y, des_LCD + 0x30, font, color, bgcolor); flag = 1;x +=font.width - 1;
 	 if(zap == 1){IPS_st7789_WriteChar(x, y, ',', font, color, bgcolor); x += font.width - 1;}
 	 IPS_st7789_WriteChar(x, y, edi_LCD + 0x30, font, color, bgcolor); flag = 1;
 	 Poz = x + font.width - 1;
  }

 void IPS_st7789_WriteF(uint16_t x, uint16_t y, float dig, FontDef font, uint16_t color, uint16_t bgcolor)
 {

	 	 int flag = 0;
	 	uint16_t zap = 2;

	 	 int32_t cc = (int32_t)(dig * 100);
	 	 Preobr_LCD (cc);
		 IPS_st7789_WriteChar(x                     , y, ' ', font, color, bgcolor);
		 IPS_st7789_WriteChar(x +     font.width - 1, y, ' ', font, color, bgcolor);
		 IPS_st7789_WriteChar(x + 2 * font.width - 1, y, ' ', font, color, bgcolor);
		 IPS_st7789_WriteChar(x + 3 * font.width - 1, y, ' ', font, color, bgcolor);
		 IPS_st7789_WriteChar(x + 4 * font.width - 1, y, ' ', font, color, bgcolor);
		 IPS_st7789_WriteChar(x + 5 * font.width - 1, y, ' ', font, color, bgcolor);



		 	 if((sts_LCD) || (zap == 5))           {IPS_st7789_WriteChar(x, y, sts_LCD + 0x30, font, color, bgcolor); flag = 1;x +=font.width - 1;}
			 if(zap == 5){IPS_st7789_WriteChar(x, y, ',', font, color, bgcolor); x += font.width - 1;}
			 if((dts_LCD) || (flag) || (zap == 4)) {IPS_st7789_WriteChar(x, y, dts_LCD + 0x30, font, color, bgcolor); flag = 1;x +=font.width - 1;}
			 if(zap == 4){IPS_st7789_WriteChar(x, y, ',', font, color, bgcolor); x += font.width - 1;}
			 if((tis_LCD) || (flag) || (zap == 3)) {IPS_st7789_WriteChar(x, y, tis_LCD + 0x30, font, color, bgcolor); flag = 1;x +=font.width - 1;}
			 if(zap == 3){IPS_st7789_WriteChar(x, y, ',', font, color, bgcolor); x += font.width - 1;}
			 if((sot_LCD) || (flag) || (zap == 2)) {IPS_st7789_WriteChar(x, y, sot_LCD + 0x30, font, color, bgcolor); flag = 1;x +=font.width - 1;}
			 if(zap == 2){IPS_st7789_WriteChar(x, y, ',', font, color, bgcolor); x += font.width - 1;}
			 if((des_LCD) || (flag) || (zap == 1)) {IPS_st7789_WriteChar(x, y, des_LCD + 0x30, font, color, bgcolor); flag = 1;x +=font.width - 1;}
			 if(zap == 1){IPS_st7789_WriteChar(x, y, ',', font, color, bgcolor); x += font.width - 1;}
			                          IPS_st7789_WriteChar(x, y, edi_LCD + 0x30, font, color, bgcolor); flag = 1;
			 Poz = x + font.width - 1;

 }


/*
Simpler (and probably slower) implementation:
static void ST7735_WriteChar(uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color) {
    uint32_t i, b, j;
    for(i = 0; i < font.height; i++) {
        b = font.data[(ch - 32) * font.height + i];
        for(j = 0; j < font.width; j++) {
            if((b << j) & 0x8000)  {
                ST7735_DrawPixel(x + j, y + i, color);
            }
        }
    }
}
*/

void IPS_st7789_WriteString(uint16_t x, uint16_t y, const char* str, FontDef font, uint16_t color, uint16_t bgcolor)
{

    while(*str) {
        if(x + font.width >= ST7789_IPS_WIDTH_240) {
            x = 0;
            y += font.height;
            if(y + font.height >= ST7789_IPS_WIDTH_240) {
                break;
            }

            if(*str == ' ') {
                // skip spaces in the beginning of the new line
                str++;
                continue;
            }
        }

        IPS_st7789_WriteChar(x, y, *str, font, color, bgcolor);
        x += font.width;
        str++;
    }
}

void IPS_st7789_WriteSIM(uint16_t x, uint16_t y, uint8_t* str,uint8_t len, FontDef font, uint16_t color, uint16_t bgcolor)
{
	uint8_t i;


    for(i = 0; i < len; i++)
    {
        if(x + font.width >= ST7789_IPS_WIDTH_240)
        {
            x = 0;
            y += font.height;
            if(y + font.height >= ST7789_IPS_WIDTH_240) {break;}

            if(*str == ' ') // skip spaces in the beginning of the new line
            {
                str++;
                continue;
            }
        }


        if(*str == 0xD) {str++;continue;}

		if((*str == 0xA) && (*(str - 1) == 0xD))
		{
			y += font.height;
			x= 0;
			if (y + font.height >= ST7789_IPS_WIDTH_240) { x= 0; y = 0; IPS_st7789_FillScreen(BLACK);}
			str++;
			  continue;
		}

		  if((*str == 'O') && (*(str + 1) == 'K') && (*(str + 2) == 0xD) && (*(str + 3) == 0xA)) {color = GREEN;}

        IPS_st7789_WriteChar(x, y, *str, font, color, bgcolor);
        x += font.width;
        str++;
    }
    Stroka = y;
    Poz = x;
}


void IPS_st7789_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    // clipping
    if((x >= ST7789_IPS_WIDTH_240) || (y >= ST7789_IPS_HEIGHT_240)) return;
    if((x + w - 1) >= ST7789_IPS_WIDTH_240) w = ST7789_IPS_WIDTH_240 - x;
    if((y + h - 1) >= ST7789_IPS_HEIGHT_240) h = ST7789_IPS_HEIGHT_240 - y;

    IPS_st7789_SetAddressWindow(x, y, x+w-1, y+h-1);

    uint8_t data[] = { color >> 8, color & 0xFF };
    HAL_GPIO_WritePin(IPS_DC_GPIO_Port, IPS_DC_Pin, GPIO_PIN_SET);
    for(y = h; y > 0; y--) {
        for(x = w; x > 0; x--) {
            HAL_SPI_Transmit(&IPS_st7789_SPI_PORT, data, sizeof(data), HAL_MAX_DELAY);
        }
    }

}


void IPS_st7789_FillScreen(uint16_t color) {

	uint32_t i;
	uint8_t data[] = { color >> 8, color & 0xFF };

	IPS_st7789_SetAddressWindow(0, 0, ST7789_IPS_HEIGHT_240, ST7789_IPS_HEIGHT_240);
	for(i = 0; i < ST7789_IPS_HEIGHT_240 * ST7789_IPS_HEIGHT_240; i++)
	{
		IPS_st7789_WriteData((uint8_t*)data,  sizeof(data));
	}

}
/*
void ST7735_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data) {
    if((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT)) return;
    if((x + w - 1) >= ST7735_WIDTH) return;
    if((y + h - 1) >= ST7735_HEIGHT) return;

    ST7735_Select();
    ST7735_SetAddressWindow(x, y, x+w-1, y+h-1);
    ST7735_WriteData((uint8_t*)data, sizeof(uint16_t)*w*h);
    ST7735_Unselect();
}
*/
void IPS_st7789_InvertColors(uint8_t invert) {
	IPS_st7789_WriteCommand(invert ? ST7789_INVON : ST7789_INVOFF);
}


/*
 * IPS_st7789_fonts.h
 *
 *  Created on: 12 ��� 2019 �.
 *      Author: vvv
 */

#ifndef IPS_ST7789_FONTS_H_
#define IPS_ST7789_FONTS_H_

#include <stdint.h>

typedef struct
{
    const uint8_t   width;
    uint8_t         height;
    const uint16_t  *data;
} FontDef;


extern FontDef Font_7x10;
extern FontDef Font_11x18;
extern FontDef Font_16x26;

#endif /* IPS_ST7789_FONTS_H_ */

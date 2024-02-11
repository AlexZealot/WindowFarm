/*
 * ssd1306.h
 *
 *  Created on: Nov 14, 2023
 *      Author: Алексей
 */

#ifndef INC_SSD1306_H_
#define INC_SSD1306_H_

#include "stm32f1xx_hal.h"
#include "ssd1306/fonts.h"

//#define	_REINIT_DISPLAY_
#define _REINIT_ON_WRITE_

typedef enum {
    ssd_stat_ok,
    ssd_stat_fail
} ssd1306_status;

//For the SSD1306, the slave
//address is either “b0111100” or “b0111101”
#define _SSD1306_DISPLAY_WIDTH	128
#define _SSD1306_DISPLAY_HEIGHT	32
#define _SSD1306_BUF_SIZE		_SSD1306_DISPLAY_WIDTH * _SSD1306_DISPLAY_HEIGHT / 8
typedef struct
{
	I2C_HandleTypeDef*	_hi2c_bus;
	uint8_t				_a_display_addr;
	int					CurrentX;
	int					CurrentY;
	uint8_t				buf[_SSD1306_BUF_SIZE];
} A_SSD1306_TypeDef;

// @brief clear display data
// @param display - display instance
// @param show - if true, shows display data after wiping
void			ssd1306_clear(A_SSD1306_TypeDef * display, uint8_t show);
// @brief display initialization
ssd1306_status	ssd1306_init(A_SSD1306_TypeDef * display, I2C_HandleTypeDef * hi2c, uint8_t ADDR);
// @brief show display buffer
void			ssd1306_show(A_SSD1306_TypeDef * display);
// @brief inverse display colors
// @param display - SSD1306 display instance
// @param invert - 0 - normal display color; 1 - inverted display color
void			ssd1306_set_inverse(A_SSD1306_TypeDef * display, uint8_t invert);
// @brief put pixel to buffer
// @param display - SSD1306 display instance
// @param x, y - coordinates
// @param col 0 - black; any other - white
void			ssd1306_put_pixel(A_SSD1306_TypeDef * display, uint8_t x, uint8_t y, uint8_t col);
// @brief put line to buffer
// @param display - SSD1306 display instance
// @param x1, y1, x2, y2 - coordinates
// @param col 0 - black; any other - white
void			ssd1306_put_line(A_SSD1306_TypeDef * display, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t col);

// @brief found on github
char			ssd1306_put_char(A_SSD1306_TypeDef * display, FontDef * font, char ch, uint8_t col);
// @brief found on github
char 			ssd1306_put_string(A_SSD1306_TypeDef * display, FontDef * font, char* str, uint8_t col);

/*CONSTANTS FOR I2C OMMUNICATION*/
#define		SSD1306_SET_CONTRAST_CONTROL		0x81
#define		SSD1306_SET_DISPLAY_NORMAL			0xA6
#define		SSD1306_SET_DISPLAY_INVERSE			0xA7
#define		SSD1306_DISPLAY_ON					0xAF
#define		SSD1306_DISPLAY_OFF					0xAE
#define 	SSD1306_SET_MUX_RATIO				0xA8
#define		SSD1306_SET_DISPLAY_OFFSET			0xD3
#define		SSD1306_SET_DISPLAY_START_LINE		0x40
#define		SSD1306_SET_SEGMENT_0				0xA0
#define		SSD1306_SET_SEGMENT_127				0xA1
#define		SSD1306_SET_COM_OUTPUT_DIR_NORM		0xC0
#define		SSD1306_SET_COM_OUTPUT_DIR_REVERSE	0xC8
#define		SSD1306_SET_COM_PINS_CONFIG			0xDA
#define		SSD1306_SET_ENTIRE_DISPLAY_ON		0xA4
#define		SSD1306_SET_ENTIRE_DISPLAY_OFF		0xA5
#define		SSD1306_SET_DISPLAY_INVERSE			0xA7
#define		SSD1306_SET_DISPLAY_CLOCK_DIVIDE	0xD5
#define		SSD1306_CHARGE_PUMP_SETUP			0x8D
#define		SSD1306_SET_PAGE_ADDR				0x22
#define		SSD1306_SET_COL_ADDR				0x21
#define		SSD1306_SET_MEMORY_ADDR_MODE		0x20
#define 	SSD1306_SET_COM_HARDWARE_CONF		0xDA
#define		SSD1306_SET_PAGE_ADDRESSING			0xB0
#define		SSD1306_SET_VCOMH					0xDB
#define		SSD1306_SET_HC_START_ADDR			0x10


#endif /* INC_SSD1306_H_ */

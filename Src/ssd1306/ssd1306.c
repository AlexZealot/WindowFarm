#include "ssd1306/ssd1306.h"
#include <string.h>
#include <stdlib.h>

#define CMD_CHECK(X)	if (X == ssd_stat_fail) return ssd_stat_fail
#define HAL_CHECK(X)	if (X != HAL_OK)	return ssd_stat_fail
uint8_t start_tr_byte = 0x00;

void ssd1306_sendCMD(A_SSD1306_TypeDef * display, uint8_t cmd){
	if (HAL_I2C_Mem_Write(display->_hi2c_bus, display->_a_display_addr, 0x00, 1, &cmd, 1, 1000) != HAL_OK){
#ifdef _REINIT_ON_WRITE_
		ssd1306_init(display, display->_hi2c_bus, display->_a_display_addr);
#endif
	}
}

void ssd1306_sendData(A_SSD1306_TypeDef * display, uint8_t * data, uint16_t len){
	if (HAL_I2C_Mem_Write(display->_hi2c_bus, display->_a_display_addr, 0x40, 1, data, len, 1000) != HAL_OK){
#ifdef _REINIT_ON_WRITE_
		ssd1306_init(display, display->_hi2c_bus, display->_a_display_addr);
#endif
	}
}

#define INV_BYTE(X)		((X & 0b00000001 << 7) | (X & 0b00000010 << 5) | (X & 0b00000100 << 3) | (X & 0b00001000 << 1) | (X & 0b00010000 >> 1) | (X & 0b00100000 >> 3) | (X & 0b01000000 >> 5) | (X & 0b10000000 >> 7))

// @brief display initialization
ssd1306_status	ssd1306_init(A_SSD1306_TypeDef * display, I2C_HandleTypeDef * hi2c, uint8_t ADDR){
	display->_hi2c_bus = hi2c;
	display->_a_display_addr = (ADDR << 1);
	memset(display->buf, 0x00, _SSD1306_BUF_SIZE);

	//А не согрешил ли я этим циклом...
	HAL_StatusTypeDef res = HAL_TIMEOUT;
    for(uint8_t i_v = 0; i_v < 5; i_v++) {
        res = HAL_I2C_IsDeviceReady(display->_hi2c_bus, display->_a_display_addr, 1, 1000);
        if(res == HAL_OK)
            break;
    }
    if (res != HAL_OK) return ssd_stat_fail;

    HAL_Delay(100);

    ssd1306_sendCMD(display, SSD1306_DISPLAY_OFF);
	ssd1306_sendCMD(display, SSD1306_SET_ENTIRE_DISPLAY_OFF);
    ssd1306_sendCMD(display, SSD1306_SET_MEMORY_ADDR_MODE);
    ssd1306_sendCMD(display, 0b00000010);
    ssd1306_sendCMD(display, SSD1306_SET_PAGE_ADDR);
    ssd1306_sendCMD(display, 0);
    ssd1306_sendCMD(display, _SSD1306_DISPLAY_HEIGHT-1);

    ssd1306_sendCMD(display, SSD1306_SET_MUX_RATIO);
    ssd1306_sendCMD(display, _SSD1306_DISPLAY_HEIGHT-1);

    ssd1306_sendCMD(display, SSD1306_SET_DISPLAY_OFFSET);
    ssd1306_sendCMD(display, 0x00);

    ssd1306_sendCMD(display, SSD1306_SET_SEGMENT_0);

    ssd1306_sendCMD(display, SSD1306_SET_COM_OUTPUT_DIR_NORM);

    ssd1306_sendCMD(display, SSD1306_SET_COM_PINS_CONFIG);
	ssd1306_sendCMD(display, 0x02);

	ssd1306_sendCMD(display, SSD1306_SET_CONTRAST_CONTROL);
	ssd1306_sendCMD(display, 0x7F);

    ssd1306_sendCMD(display, SSD1306_SET_DISPLAY_NORMAL);

    ssd1306_sendCMD(display, SSD1306_SET_DISPLAY_CLOCK_DIVIDE);
    ssd1306_sendCMD(display, 0xF0);

    ssd1306_sendCMD(display, SSD1306_CHARGE_PUMP_SETUP);
    ssd1306_sendCMD(display, 0x14);

    ssd1306_sendCMD(display, SSD1306_SET_COM_HARDWARE_CONF);
    ssd1306_sendCMD(display, 0b00000010);

    ssd1306_sendCMD(display, SSD1306_SET_VCOMH);
    ssd1306_sendCMD(display, 0x20);

	ssd1306_sendCMD(display, SSD1306_SET_ENTIRE_DISPLAY_ON);
    ssd1306_sendCMD(display, SSD1306_DISPLAY_ON);
    return ssd_stat_ok;
}

// @brief show display buffer
void			ssd1306_show(A_SSD1306_TypeDef * display){
#ifdef _REINIT_DISPLAY_
	ssd1306_init(display, display->_hi2c_bus, display->_a_display_addr);
#endif
	for (uint16_t i = 0; i < _SSD1306_DISPLAY_HEIGHT/8; i++){
		ssd1306_sendCMD(display, SSD1306_SET_PAGE_ADDRESSING + i);
		ssd1306_sendCMD(display, SSD1306_SET_HC_START_ADDR);
		ssd1306_sendData(display, &display->buf[i*_SSD1306_DISPLAY_WIDTH], _SSD1306_DISPLAY_WIDTH);
	}
}
// @brief inverse display colors
// @param display - SSD1306 display instance
// @param invert - 0 - normal display color; 1 - inverted display color
void			ssd1306_set_inverse(A_SSD1306_TypeDef * display, uint8_t invert){
#ifdef _REINIT_DISPLAY_
	ssd1306_init(display, display->_hi2c_bus, display->_a_display_addr);
#endif
	ssd1306_sendCMD(display, invert?SSD1306_SET_DISPLAY_INVERSE:SSD1306_SET_DISPLAY_NORMAL);
}
// @brief inverse display colors
// @param display - SSD1306 display instance
// @param x, y - coordinates
// @param col 0 - black; any other - white
void			ssd1306_put_pixel(A_SSD1306_TypeDef * display, uint8_t x, uint8_t y, uint8_t col){
#ifdef _REINIT_DISPLAY_
	ssd1306_init(display, display->_hi2c_bus, display->_a_display_addr);
#endif
	if (col) display->buf[(y >> 3)*_SSD1306_DISPLAY_WIDTH + x] |=  (1 << (y % 8));
	else	 display->buf[(y >> 3)*_SSD1306_DISPLAY_WIDTH + x] &= ~(1 << (y % 8));
}

// @brief put line to buffer
// @param display - SSD1306 display instance
// @param x, y, x2, y2 - coordinates
// @param col 0 - black; any other - white
void			ssd1306_put_line(A_SSD1306_TypeDef * display, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t col){
	if (x1 >= _SSD1306_DISPLAY_WIDTH) return;
	if (x2 >= _SSD1306_DISPLAY_WIDTH) return;
	if (y1 >= _SSD1306_DISPLAY_HEIGHT) return;
	if (y2 >= _SSD1306_DISPLAY_HEIGHT) return;

    int32_t deltaX = abs(x2 - x1);
    int32_t deltaY = abs(y2 - y1);
    int32_t signX = ((x1 < x2) ? 1 : -1);
    int32_t signY = ((y1 < y2) ? 1 : -1);
    int32_t error = deltaX - deltaY;
    int32_t error2;

    ssd1306_put_pixel(display, x2, y2, col);

    while((x1 != x2) || (y1 != y2)) {
    	ssd1306_put_pixel(display, x1, y1, col);
        error2 = error * 2;
        if(error2 > -deltaY) {
            error -= deltaY;
            x1 += signX;
        }

        if(error2 < deltaX) {
            error += deltaX;
            y1 += signY;
        }
    }
    return;
}

char			ssd1306_put_char(A_SSD1306_TypeDef * display, FontDef * font, char ch, uint8_t col){
    uint32_t i, b, j;

    // Check if character is valid
    if (ch < 32 || ch > 126)
        return 0;

    // Check remaining space on current line
    if (_SSD1306_DISPLAY_WIDTH < (display->CurrentX + font->FontWidth) ||
        _SSD1306_DISPLAY_HEIGHT < (display->CurrentY + font->FontHeight))
    {
        // Not enough space on current line
        return 0;
    }

    // Use the font to write
    for(i = 0; i < font->FontHeight; i++) {
        b = font->data[(ch - 32) * font->FontHeight + i];
        for(j = 0; j < font->FontWidth; j++) {
            if((b << j) & 0x8000)  {
            	ssd1306_put_pixel(display, display->CurrentX + j, (display->CurrentY + i), col);
            } else {
            	ssd1306_put_pixel(display, display->CurrentX + j, (display->CurrentY + i), !col);
            }
        }
    }

    // The current space is now taken
    display->CurrentX += font->FontWidth;

    // Return written char for validation
    return ch;
}

char 			ssd1306_put_string(A_SSD1306_TypeDef * display, FontDef * font, char* str, uint8_t col){
    while (*str) {
        if (ssd1306_put_char(display, font, *str, col) != *str) {
            return *str;
        }
        str++;
    }
    return *str;
}

void			ssd1306_clear(A_SSD1306_TypeDef * display, uint8_t show){
	memset(&display->buf[0], 0, _SSD1306_BUF_SIZE);
	if (show)
		ssd1306_show(display);
}

/*
 * DTH11.h
 *
 *  Created on: Nov 15, 2023
 *      Author: Алексей
 */

#ifndef INC_DHT11_DHT11_H_
#define INC_DHT11_DHT11_H_

#include "stm32f1xx_hal.h"

#define		DHT_SAFE_DELAY		3000

typedef struct
{
	GPIO_TypeDef	*port;
	uint16_t		pin;

	float			T;
	float			H;

	uint8_t 		hH;
	uint8_t 		lH;
	uint8_t 		hT;
	uint8_t 		lT;

	uint32_t		lastRequest;
} A_DHT11_TypeDef;

void dht11_init(A_DHT11_TypeDef * dht, GPIO_TypeDef * port, uint16_t pin);
void dht11_read(A_DHT11_TypeDef * dht);
void dht11_read_unsafe(A_DHT11_TypeDef * dht);

#endif /* INC_DHT11_DHT11_H_ */

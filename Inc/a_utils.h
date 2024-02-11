/*
 * a_utils.h
 *
 *  Created on: Mar 11, 2023
 *      Author: Алексей
 *	Copypaste and analogs to arduino core functions
 */

#ifndef INC_A_UTILS_H_
#define INC_A_UTILS_H_

#include "stm32f1xx_hal.h"

//#define _USE_DELAY_US_

//#define _USE_DELAY_US_
#ifdef _USE_DELAY_US_
/*
 *	Для использования delay_us необходимо настроить один из таймеров на частоту в 1МГц и передать его хэндл в эту функцию
 */
void _init_delay_us(TIM_HandleTypeDef * _htim);

/*
 * Ардуиновский аналог delay_us
 */
void delay_us(uint32_t us);
/*
 * Ардуиновский аналог micros
 */
uint32_t micros();
#endif

__STATIC_INLINE void _DWT_Init(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // разрешаем использовать счётчик
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;   // запускаем счётчик
}

__STATIC_INLINE void _delay_us(uint32_t us)
{
	uint32_t us_count_tic =  us * (SystemCoreClock / 1000000U);
	DWT->CYCCNT = 0U;
	while(DWT->CYCCNT < us_count_tic);
}

__STATIC_INLINE uint32_t _micros(void){
	return  DWT->CYCCNT / (SystemCoreClock / 1000000U);
}
/*
 * линейная интерпояция
 */
long map(long x, long in_min, long in_max, long out_min, long out_max);
long min(long a, long b);
long max(long a, long b);
long constrain(long a, long a_min, long a_max);

#endif /* INC_A_UTILS_H_ */

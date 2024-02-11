/*
 * a_utils.c
 *
 *  Created on: Mar 11, 2023
 *      Author: Алексей
 */


#include "a_utils.h"

#ifdef _USE_DELAY_US_
TIM_HandleTypeDef * us_tim;

/*
 *	Для использования delay_us необходимо настроить один из таймеров на частоту в 1МГц и передать его хэндл в эту функцию
 */
void _init_delay_us(TIM_HandleTypeDef * _htim){
	us_tim = _htim;
	HAL_TIM_Base_Start(us_tim);
}

/*
 * Ардуиновский аналлог delay_us
 */
void delay_us(uint32_t us){
	uint32_t m = us_tim->Instance->CNT;
	while (m - us_tim->Instance->CNT < us){};
}

uint32_t micros(){
	return us_tim->Instance->CNT;
}

#endif


/*
 * линейная интерполяция
 */
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

long min(long a, long b){
	return ((a<b)?a:b);
}

long max(long a, long b){
	return ((a>b)?a:b);
}

long constrain(long a, long a_min, long a_max){
	long t;
	if (a_min > a_max){
		t = a_min;
		a_min = a_max;
		a_max = t;
	}
	t = (a<a_min)?(a_min):((a>a_max)?a_max:a);
	return t;
}

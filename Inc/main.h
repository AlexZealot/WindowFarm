/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum {
	ADC_HUM_1,
	ADC_HUM_2,
	ADC_PHOTO
} adc_pin_t;

typedef struct {
	uint8_t 		mode;
	uint32_t		startedTime;
} display_struct_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
uint32_t	analogRead(adc_pin_t pin);
void		ADC_Select_CH_HUM1(void);
void		ADC_Select_CH_HUM2(void);
void		ADC_Select_CH_LIGHT_SENSOR(void);
void		WaterCheck(void);
void		UpdDisplay(void);
uint8_t 	isItDayNow();
uint32_t 	timeLeftTo(uint8_t day);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LIGHT_SENSOR_Pin GPIO_PIN_3
#define LIGHT_SENSOR_GPIO_Port GPIOA
#define HUM_1_Pin GPIO_PIN_4
#define HUM_1_GPIO_Port GPIOA
#define HUM_2_Pin GPIO_PIN_5
#define HUM_2_GPIO_Port GPIOA
#define DHT11_Pin GPIO_PIN_10
#define DHT11_GPIO_Port GPIOB
#define LEDS_Pin GPIO_PIN_4
#define LEDS_GPIO_Port GPIOB
#define WATER_1_Pin GPIO_PIN_5
#define WATER_1_GPIO_Port GPIOB
#define WATER_2_Pin GPIO_PIN_8
#define WATER_2_GPIO_Port GPIOB
#define HEATER_Pin GPIO_PIN_9
#define HEATER_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
//#define __DEBUG_HUM__						//Отладочные интервалы для датчиков влажности

#define	DISPLAY_DELAY		5000			//Частота смены данных дисплея
#define	DISPL_MODE_DHT		0				//Режим дисплея - данные с DHT11
#define DISPL_MODE_HUM		1				//Режим дисплея - данные с датчиков влажности в попугаях
#define DISPL_MODE_TIME		2				//Режим дисплея - часы

#define	PUMP_1_LIMIT_ON		1750			//Порог включения помпы 1
#define PUMP_1_LIMIT_OFF	1500			//Порог выключения помпы 1

#define	PUMP_2_LIMIT_ON		1850			//Порог включения помпы 2
#define PUMP_2_LIMIT_OFF	1600			//Порог выключения помпы 2

#ifndef __DEBUG_HUM__
	#define	PUMP_MAX_TIME_ON	4000			//Максимальное время работы помпы
	#define PUMP_MIN_TIME_OFF	180000			//Минимальное время работы помпы между двумя включениями
#else
	#define	PUMP_MAX_TIME_ON	1000			//Максимальное время работы помпы
	#define PUMP_MIN_TIME_OFF	30000			//Минимальное время работы помпы между двумя включениями
#endif

#define	HUM_DELAY			1000			//как часто считываем данные с датчиков влажности почвы

#define DAY_H_START			6				//День начинается в 6:00
#define DAY_M_START			0				//День начинается в 6:00
#define DAY_H_END			21				//День закончится в 22:00
#define DAY_M_END			59				//День закончится в 22:00
#define USE_RTC

//#define NO_HUM1_SENSOR

//#define __DEBUG_LIGHT__						//Включить ли отладочные интервалы для работы с LED

#ifdef __DEBUG_LIGHT__
	#define LIGHT_T_ON		20000			//Световой день для растений в отладочном режиме
	#define LIGHT_T_OFF		10000			//Ночь для растений в отладочном режиме
#else
	#define LIGHT_T_ON		57600000		//Световой день для растений в нормальном режиме
	#define LIGHT_T_OFF		28800000		//Ночь для растений в нормальном режиме
#endif

#define LIGHT_DELAY			1000			//Периодичность опроса датчика света
#define LIGHT_TIME			LIGHT_T_ON		//Сколько длится световой день для растений. 	norm val: 16h = 960min 		= 57 600sec 					= 57 600 000ms		debug val: 20 000
#define DARK_TIME			LIGHT_T_OFF		//Сколько длится ночь для растений. 			norm val: 24h - LIGHT_TIME 	= 86 400 000ms - 57 600 000ms 	= 28 800 000ms		debug val: 10 000
#define LIGHT_SW_ON			2000			//Порог включения подсветки
#define LIGHT_SW_OFF		1500			//Порог выключения подсветки
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

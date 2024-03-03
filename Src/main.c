/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "rtc.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "ssd1306/ssd1306.h"
#include "ssd1306/fonts.h"
#include "DHT11/DHT11.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#ifdef _USB_DEBUG_
char 					usb_str[64];
#endif
//Датчики
A_DHT11_TypeDef			dht;
A_SSD1306_TypeDef		display;
uint16_t				hum1, hum2;
uint16_t				light_val;
uint32_t				lightTick;

//Переключение дисплея
display_struct_t		d_struct;				//структура с сотоянием дисплея
char					display_str[64];		//буфер строки для вывода текста на экран

//Состояние помпы
uint8_t					p1state = 0;			//состояние помпы 1
uint8_t					p2state = 0;			//состояние помпы 2
uint32_t				hum_time = 0;			//отслеживаем периодичность опроса датчиков влажности
uint32_t				pump_1_started_at = 0;  //помпа 1 включилась в это время
uint32_t				pump_1_stopped_at = 0;	//помпа 1 отлючилась в это время
uint32_t				pump_2_started_at = 0;	//помпа 2 включилась в это время
uint32_t				pump_2_stopped_at = 0;	//помпа 2 отлючилась в это время

//Обработчик света
uint8_t					light_state = 0;		//0 = ночь; 1 = день
uint32_t				light_started = 0;		//когда запустился текущий период дня
#ifndef USE_RTC
uint8_t					light_cycle = 0;		//Запущен ли цикл работы с освещением
#endif
uint32_t				light_time_left = 0;	//Сколько осталось до переключения день/ночь

//RTC
RTC_TimeTypeDef	sTime = {0,};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(WATER_1_GPIO_Port, WATER_1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(WATER_2_GPIO_Port, WATER_2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LEDS_GPIO_Port, LEDS_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(HEATER_GPIO_Port, HEATER_Pin, GPIO_PIN_RESET);

  dht11_init(&dht, DHT11_GPIO_Port, DHT11_Pin);
  dht11_read(&dht);
  HAL_Delay(1000);
  ssd1306_init(&display, &hi2c1, 0b0111100);
  ssd1306_clear(&display, 1);
  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  hum1 = analogRead(ADC_HUM_1);
  hum2 = analogRead(ADC_HUM_2);
  light_val = analogRead(ADC_PHOTO);
#ifdef USE_RTC
  light_state = isItDayNow();
#else
  light_state = light_val < LIGHT_SW_OFF?1:0;
#endif

  d_struct.mode = DISPL_MODE_DHT;
  hum_time = light_started = lightTick = d_struct.startedTime = HAL_GetTick();

  UpdDisplay();
  while (1)
  {
	  /*Опрос датчиков влажности почвы*/
	  if (HAL_GetTick() - hum_time > HUM_DELAY){
		  /*Поливаем?*/
  		  hum1 = analogRead(ADC_HUM_1);
  		  hum2 = analogRead(ADC_HUM_2);
  		  WaterCheck();
  		  UpdDisplay();
		  hum_time = HAL_GetTick();
	  }

	  /*Переключаем дисплей каждые DISPLAY_DELAY мс*/
	  if (HAL_GetTick() - d_struct.startedTime > DISPLAY_DELAY){
		  d_struct.mode = (d_struct.mode + 1)%3;
		  UpdDisplay();
		  d_struct.startedTime = HAL_GetTick();
	  }

#ifndef USE_RTC
	  if (light_cycle){
		  if (HAL_GetTick() - lightTick > LIGHT_DELAY){
			  light_val = analogRead(ADC_PHOTO);

			  if (d_struct.mode == DISPL_MODE_DHT)
				  UpdDisplay();

			  if (light_state){ //день
				  if (HAL_GetTick() - light_started > LIGHT_TIME){//день закончился, включаем ночь
					  light_started = HAL_GetTick();
					  light_state = 0;
				  } else {
					  //обработка данных с датчика света
					  if (light_val < LIGHT_SW_OFF){
						  LEDS_GPIO_Port->BSRR = (uint32_t)LEDS_Pin << 16u;
					  }
					  if (light_val > LIGHT_SW_ON){
						  LEDS_GPIO_Port->BSRR = (uint32_t)LEDS_Pin;
					  }
				  }
			  } else { // ночь
				  HAL_GPIO_WritePin(LEDS_GPIO_Port, LEDS_Pin, GPIO_PIN_RESET);
				  if (HAL_GetTick() - light_started > DARK_TIME){//ночь закончилась. Включаем день
					  light_started = HAL_GetTick();
					  light_state = 1;
				  }
			  }

			  lightTick = HAL_GetTick();
		  }
	  } else {
		  if (HAL_GetTick() - lightTick > LIGHT_DELAY){
			  light_val = analogRead(ADC_PHOTO);

			  if (d_struct.mode == DISPL_MODE_DHT)
				  UpdDisplay();

			  if (light_state){ // день
				  if ((light_val > LIGHT_SW_ON) || (HAL_GetTick() - light_started > LIGHT_TIME)){
					  light_cycle 		= 1;				//включим цикл работы
					  light_started		= HAL_GetTick();	//когда включился цикл
					  light_state 		= 0;				//включаем ночь
				  }
			  } else { // ночь
				  if ((light_val < LIGHT_SW_OFF) || (HAL_GetTick() - light_started > DARK_TIME)){
					  light_cycle 		= 1;				//включим цикл работы
					  light_started		= HAL_GetTick();	//когда включился цикл
					  light_state 		= 1;				//включаем день
				  }
			  }

			  lightTick = HAL_GetTick();
		  }
	  }
#else
	  if (HAL_GetTick() - lightTick > LIGHT_DELAY){
		  light_val = analogRead(ADC_PHOTO);

		  if (d_struct.mode == DISPL_MODE_DHT)
			  UpdDisplay();

		  light_state = isItDayNow();

		  if (light_state){ //день
			  if (light_val < LIGHT_SW_OFF){
				  LEDS_GPIO_Port->BSRR = (uint32_t)LEDS_Pin << 16u;
			  }
			  if (light_val > LIGHT_SW_ON){
				  LEDS_GPIO_Port->BSRR = (uint32_t)LEDS_Pin;
			  }
		  } else { // ночь
			  LEDS_GPIO_Port->BSRR = (uint32_t)LEDS_Pin << 16u;
		  }
		  lightTick = HAL_GetTick();
	  }
#endif

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void	ADC_Select_CH_HUM1(void){
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

void	ADC_Select_CH_HUM2(void){
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

void	ADC_Select_CH_LIGHT_SENSOR(void){
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

//@brief  Для простоты понимания названо аналогично ардуиновской функции работы с АЦП. Внимание, функция блокирующая выполнение остальной программы
//@param  pin С какого пина читать значения. ADC_HUM_1, ADC_HUM_2, ADC_PHOTO
//@return  Данные АЦП
uint32_t	analogRead(adc_pin_t pin){
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
	switch (pin){
		case ADC_HUM_1:
			sConfig.Channel = ADC_CHANNEL_4;
			break;

		case ADC_HUM_2:
			sConfig.Channel = ADC_CHANNEL_5;
			break;

		case ADC_PHOTO:
			sConfig.Channel = ADC_CHANNEL_3;
			break;
	}
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	uint32_t res = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	return res;
}

void	WaterCheck(void){
#ifdef NO_HUM1_SENSOR
	if ((hum2 > PUMP_1_LIMIT_ON) && ((HAL_GetTick() - pump_1_stopped_at) >= PUMP_MIN_TIME_OFF)){
#else
	if ((hum1 > PUMP_1_LIMIT_ON) && ((HAL_GetTick() - pump_1_stopped_at) >= PUMP_MIN_TIME_OFF)){
#endif
		if (!p1state) {
			p1state = 1;
			pump_1_started_at = HAL_GetTick();
		}
		WATER_1_GPIO_Port->BSRR = (uint32_t)WATER_1_Pin;
	}

#ifdef NO_HUM1_SENSOR
	if ((hum2 < PUMP_1_LIMIT_OFF) || ((HAL_GetTick() - pump_1_started_at) >= PUMP_MAX_TIME_ON)){
#else
	if ((hum1 < PUMP_1_LIMIT_OFF) || ((HAL_GetTick() - pump_1_started_at) >= PUMP_MAX_TIME_ON)){
#endif
		if (p1state){
			p1state = 0;
			pump_1_stopped_at = HAL_GetTick();
		}
		WATER_1_GPIO_Port->BSRR = (uint32_t)WATER_1_Pin << 16u;
	}

	if ((hum2 > PUMP_2_LIMIT_ON) && ((HAL_GetTick() - pump_2_stopped_at) >= PUMP_MIN_TIME_OFF)){
		if (!p2state){
			p2state = 1;
			pump_2_started_at = HAL_GetTick();
		}
		WATER_2_GPIO_Port->BSRR = (uint32_t)WATER_2_Pin;
	}

	if ((hum2 < PUMP_2_LIMIT_OFF) || ((HAL_GetTick() - pump_2_started_at) >= PUMP_MAX_TIME_ON)){
		if (p2state){
			p2state = 0;
			pump_2_stopped_at = HAL_GetTick();
		}
		WATER_2_GPIO_Port->BSRR = (uint32_t)WATER_2_Pin << 16u;
	}
}

void	UpdDisplay(void){
	switch (d_struct.mode){
		case DISPL_MODE_DHT:
			ssd1306_clear(&display, 0);

		  	dht11_read(&dht);

		  	display.CurrentX = 5;
		  	display.CurrentY = 6;
		  	sprintf(display_str, "T: %u.%u L: %u %s", dht.hT, dht.lT, light_val, light_state?"D":"N");
		  	ssd1306_put_string(&display, &Font_6x8, display_str, 0xFF);

		  	display.CurrentX = 5;
		  	display.CurrentY = 16;
#ifndef USE_RTC
		  	if (!light_cycle)
		  		sprintf(display_str, "H: %u.%u%%", dht.hH, dht.lH);
		  	else {
		  		light_time_left = light_state?LIGHT_TIME:DARK_TIME;
		  		light_time_left -= (HAL_GetTick() - light_started);
		  		light_time_left /= 60000;
		  		sprintf(display_str, "H: %u.%u%% TL: %lu m", dht.hH, dht.lH, light_time_left);
		  	}
#else
		  	light_time_left = timeLeftTo(isItDayNow());
		  	sprintf(display_str, "H: %u.%u%% TL: %lu m", dht.hH, dht.lH, light_time_left);
#endif
		  	ssd1306_put_string(&display, &Font_6x8, display_str, 0xFF);
		  	ssd1306_show(&display);
		  	break;

	  	case DISPL_MODE_HUM:
	  		ssd1306_clear(&display, 0);

	  		display.CurrentX = 5;
	  		display.CurrentY = 6;
	  		sprintf(display_str, "H1: %u  %s", hum1, p1state?"ON":"OFF");
	  		ssd1306_put_string(&display, &Font_6x8, display_str, 0xFF);

	  		display.CurrentX = 5;
	  		display.CurrentY = 16;
	  		sprintf(display_str, "H2: %u  %s", hum2, p2state?"ON":"OFF");
	  		ssd1306_put_string(&display, &Font_6x8, display_str, 0xFF);
	  		ssd1306_show(&display);
		  	break;

	  	case DISPL_MODE_TIME:
	  		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	  		ssd1306_clear(&display, 0);

	  		display.CurrentX = 15;
	  		display.CurrentY = 11;
	  		sprintf(display_str, "%02u:%02u (%s)", sTime.Hours, sTime.Minutes, isItDayNow()?"DAY":"NIGHT");
	  		ssd1306_put_string(&display, &Font_6x8, display_str, 0xFF);
	  		ssd1306_show(&display);
	  		break;
	}
}

uint8_t 	isItDayNow(){
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	return ((sTime.Hours >= DAY_H_START) && (sTime.Minutes >= DAY_M_START)) && ((sTime.Hours < DAY_H_END) || ((sTime.Hours == DAY_H_END) && (sTime.Minutes <= DAY_M_END)))?1:0;
}

uint32_t 	timeLeftTo(uint8_t day){
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	if (day){
		if (sTime.Hours != (DAY_H_END-1))
			return (60 - sTime.Minutes) + DAY_M_END + (DAY_H_END - (sTime.Hours + 1)) * 60;
		else
			return (60 - sTime.Minutes);
	} else {
		if (sTime.Hours > DAY_H_START) {
			return ((60 - sTime.Minutes) + (24 - (sTime.Hours + 1)) * 60 + (DAY_H_START * 60) + DAY_M_START);
		} else {
			return ((60 - sTime.Minutes) + DAY_M_START + (DAY_H_START - (sTime.Hours + 1)) * 60);
		}
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

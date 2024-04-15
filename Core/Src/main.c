/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "trace.h"
#include "delay.h"
#include "key.h"
#include "led.h"
#include <stdio.h>
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
static uint8_t flag=0;
ADC_ChannelConfTypeDef sConfig = {0};
u8 smg[17]={0xc0,0xf9,0xa4,0xb0,0x99,0x92,0x82,0xf8,\
0x80,0x90,0x88,0x83,0xc6,0xa1,0x86,0x8e};

uint8_t LED7Code[16]={0x3F,0X06,0X5B,0X4F,0X66,0X6D,0X7D,0X07,
	0X7F,0X6F,0X77,0X7C,0X39,0X5E,0X79,0X71};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
u16 Get_Adc_Average(uint32_t channel,u8 times);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//delay_ms(10);
	u1_printf("称重模式： ");
	if(HAL_GPIO_ReadPin(KEY2_GPIO_Port,GPIO_PIN_13) != RESET)
	{
		flag = (~flag )&0x01;
		if (flag) {
			u1_printf("kg mode !\n");
		} else {
			u1_printf("g mode !\n");
		}
		
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	u16 adcvalue = 0;
	long press_value = 0;//原始重量	
	u8 ge=0,shi=0,bai=0,qian=0;//数据
	float kg_value = 0;
	uint16_t temp_value = 0;
	int cnt = 0;

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
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	LED_Init();
	delay_init(72);
	u1_printf("system is running!!!\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//压力信号---数据
		
		if (cnt == 0){
			adcvalue = Get_Adc_Average(ADC_CHANNEL_0,5);
			press_value = adcvalue/4096.0 *500;//转换压力信号为0-500//  0-500g
		}
		cnt++;
		if (cnt == 10) cnt = 0;

    /* USER CODE BEGIN 3 */
		//输出段选
		ge = press_value%10;//取出个位
		
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(GPIOC,LED7Code[ge],GPIO_PIN_RESET);
		//GPIOC->ODR = smg[ge];
		HAL_Delay(10);
		HAL_GPIO_WritePin(GPIOC,0XFF,GPIO_PIN_SET);
		
		shi = press_value%100/10;//十位
		
		
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(GPIOC,LED7Code[shi],GPIO_PIN_RESET);
		//GPIOC->ODR = smg[shi];
		HAL_Delay(10);
		HAL_GPIO_WritePin(GPIOC,0XFF,GPIO_PIN_SET);
		
		bai = press_value%1000/100;//百位		
		
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
		
		HAL_GPIO_WritePin(GPIOC,LED7Code[bai],GPIO_PIN_RESET);
		//GPIOC->ODR = smg[bai];
		HAL_Delay(10);
		HAL_GPIO_WritePin(GPIOC,0XFF,GPIO_PIN_SET);
		
		qian = press_value%10000/1000;//千位 == 1kg
		
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);

		HAL_GPIO_WritePin(GPIOC,LED7Code[qian],GPIO_PIN_RESET);
		if (flag == 1 && qian == 0) {
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
		}
		
		//GPIOC->ODR = smg[qian];
		HAL_Delay(10);
		HAL_GPIO_WritePin(GPIOC,0XFF,GPIO_PIN_SET);
		

		if(flag == 0 && temp_value != press_value )
		{
			
			temp_value = press_value;
			u1_printf("称重传感器---重量值为 %d g\r\n",press_value);

		}
		
		if(flag == 1)
		{
			if(temp_value != press_value)	
			{
				temp_value = press_value;
				kg_value = (float)press_value/1000;
				u1_printf("称重传感器---重量值为 %.3f kg\r\n",kg_value);
			}
		}
		HAL_Delay(1);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
u16 Get_Adc(uint32_t channel)
{
		/**Configure Regular Channel 
		*/
		sConfig.Channel = channel;
		sConfig.Rank = ADC_REGULAR_RANK_1;
		sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
	
	HAL_ADC_Start(&hadc1);                               //开启ADC
	
	HAL_ADC_PollForConversion(&hadc1,10);                //轮询转换
 
	return (u16)HAL_ADC_GetValue(&hadc1);	
}

u16 Get_Adc_Average(uint32_t channel,u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(channel);
		HAL_Delay(5);
	}
	return temp_val/times;
} 
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

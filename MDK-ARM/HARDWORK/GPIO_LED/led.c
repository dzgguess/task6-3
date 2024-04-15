 #include "led.h"
#include "delay.h"
#include "gpio.h"


void LED_Init(void) //IO��ʼ��
{ 
 	GPIO_InitTypeDef GPIO_InitStructure;
 
 	__HAL_RCC_GPIOA_CLK_ENABLE();//ʹ��PORTʱ��
	__HAL_RCC_GPIOC_CLK_ENABLE();//ʹ��PORTʱ��
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitStructure.Pin  = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;//KEY1-KEY25
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP; 		 //�������
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;		 //IO���ٶ�Ϊ50MHz
 	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIO  	
	
	GPIO_InitStructure.Pin  = \
	GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;//KEY1-KEY25
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP; 		 //�������
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;		 //IO���ٶ�Ϊ50MHz
 	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��GPIO
	
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|\
										GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8,GPIO_PIN_SET);
}


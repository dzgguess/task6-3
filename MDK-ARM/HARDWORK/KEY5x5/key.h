#ifndef __KEY_H
#define __KEY_H	 

#include "stm32f1xx_hal.h"	 


#define KEY_CLO0_OUT_LOW  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET) 
#define KEY_CLO1_OUT_LOW  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET)
#define KEY_CLO2_OUT_LOW  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET)
#define KEY_CLO3_OUT_LOW  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET)
#define KEY_CLO4_OUT_LOW  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET)

#define KEY_CLO0_OUT_HIGH  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET) 
#define KEY_CLO1_OUT_HIGH  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET)
#define KEY_CLO2_OUT_HIGH  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET)
#define KEY_CLO3_OUT_HIGH  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET)
#define KEY_CLO4_OUT_HIGH  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET)


char key_row_scan(void);
char key_scan(void);
#endif

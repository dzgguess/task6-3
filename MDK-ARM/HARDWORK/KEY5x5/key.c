#include "key.h"
#include "delay.h"
#include "gpio.h"

/***
 *函数名：key_row_scan
 *功  能：按键行扫描
 *返回值：1~5，对应1~5行按键位置
 */

char key_row_scan(void)
{
	char key_num = 0;
	if(  HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7) == 0 )
	{ 
			delay_ms(10);
	   if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7) == 0)  //消抖
		key_num = 1;
	}//判断该列第1行按键按下
		if(  HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_8) == 0 )
	{ 
			delay_ms(10);
	   if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_8) == 0)  //消抖
		key_num = 2;
	}//判断该列第2行按键按下
		if(  HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9) == 0 )
	{ 
		delay_ms(10);
	   if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9) == 0)   //消抖
		key_num = 3;
	}//判断该列第3行按键按下
		if(  HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10) == 0 )
	{ 
		delay_ms(10);
	   if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10) == 0)  //消抖
		key_num = 4;
	}//判断该列第4行按键按下
		if(  HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11) == 0 )
	{ 
		delay_ms(10);
	   if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11) == 0)   //消抖
		key_num = 5;
	}//判断该列第5行按键按下
		return key_num;
}

/***
 *函数名：key_scan
 *功  能：5*5按键扫描
 *返回值：1~25，对应25个按键
 */
char key_scan()
{
	char Key_Num=0;
	char row_num = 0;
	KEY_CLO0_OUT_LOW;        
	if(  (row_num=key_row_scan()) != 0 )
	{ 
		while(key_row_scan() != 0);  //消抖
		Key_Num = 10+row_num;
	}
	KEY_CLO0_OUT_HIGH;
			KEY_CLO1_OUT_LOW;        
	if(  (row_num=key_row_scan()) != 0 )
	{ 
		while(key_row_scan() != 0);  //消抖
		Key_Num =20+row_num;
	}
	KEY_CLO1_OUT_HIGH;
			KEY_CLO2_OUT_LOW;        
	if(  (row_num=key_row_scan()) != 0 )
	{ 
		while(key_row_scan() != 0);  //消抖
		Key_Num = 30+row_num;
	}
	KEY_CLO2_OUT_HIGH;
			KEY_CLO3_OUT_LOW;        
	if(  (row_num=key_row_scan()) != 0 )
	{ 
		while(key_row_scan() != 0);  //消抖
		Key_Num = 40+row_num;
	}
	KEY_CLO3_OUT_HIGH;
			KEY_CLO4_OUT_LOW;        
	if(  (row_num=key_row_scan()) != 0 )
	{ 
		while(key_row_scan() != 0);  //消抖
	   Key_Num = 50+row_num;
	}
	KEY_CLO4_OUT_HIGH;
		return Key_Num;
}


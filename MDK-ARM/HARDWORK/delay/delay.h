#ifndef __DELAY_H
#define __DELAY_H 		

#include "stm32f1xx_hal.h"

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;

#define SYSTEM_SUPPORT_OS 0
	 
void delay_init(u8 SYSCLK);
void delay_ms(u16 nms);
void delay_us(u32 nus);

#endif






























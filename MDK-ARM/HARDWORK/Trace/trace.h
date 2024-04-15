#ifndef _trace_h_ 
#define _trace_h_

#include "stm32f1xx_hal.h"

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;


extern UART_HandleTypeDef huart1;

void sendstring(u8 *pdata);
void u1_printf(char* fmt,...);  

#endif

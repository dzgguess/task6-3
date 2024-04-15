#include "trace.h"
#include "stdarg.h"
#include "stdio.h"

//UART_HandleTypeDef huart1;

void sendonebyte(u8 byte)
{
	HAL_UART_Transmit(&huart1, &byte, 1, 1000);
}

void sendstring(u8 *pdata)
{
	while(*pdata != '\0')
	{
		sendonebyte(*pdata);
		pdata++;
	}
}

void u1_printf(char* fmt,...)  
{
	  char buffer[200];
	  uint16_t i;
	  va_list ap;
	  va_start(ap,fmt);          			//ap指向fmt的地址
	  i = vsprintf(buffer,fmt,ap);	                //vsprintf返回数组的长度
	  va_end(ap);

	  HAL_UART_Transmit(&huart1,(uint8_t *)buffer,i,0X00FF);
}

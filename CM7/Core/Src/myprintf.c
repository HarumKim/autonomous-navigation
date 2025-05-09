/*
 * myprintf.c
 *
 *  Created on: Oct 1, 2022
 *      Author: aavila
 */


#include "main.h"
#include "myprintf.h"
extern UART_HandleTypeDef huart3;
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE {
  /* write a character to the USART3 and Loop until the end of transmission*/
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

void printfRTC(uint8_t* buff){
	printf("Day:%x, Month:%x, Year:20%x, Hour:%x, Minutes:%x, Seconds:%x\n\r",buff[4],buff[5],buff[6],buff[2],buff[1],buff[0]);
}

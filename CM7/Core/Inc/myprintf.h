/*
 * myprintf.h
 *
 *  Created on: Oct 1, 2022
 *      Author: aavila
 */

#ifndef INC_MYPRINTF_H_
#define INC_MYPRINTF_H_
#include <stdio.h>
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

void printfRTC(uint8_t *);
#endif /* INC_MYPRINTF_H_ */


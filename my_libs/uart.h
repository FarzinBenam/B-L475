#ifndef _UART_H
#define _UART_H


#include "stm32l4xx.h"                  // Device header
#include "fcmd.h"
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>




#define write     _usart1_printf
#define terminal  printf









void	USART1_Config	  (void);
void	USART3_Config   (void);
void	_usart1_send_b	(int ch);
void	_usart1_send_s	(const char Message[]);
void	_usart1_printf	(const char *format, ...);
int		read						(void);
void	_usart3_send_b  (int ch);
void	_usart3_send_s  (const char Message[]);


#endif
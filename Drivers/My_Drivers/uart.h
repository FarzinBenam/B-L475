#ifndef _UART_H
#define _UART_H


#include "main.h"


/*******************************************************************************
 * defintions
 *******************************************************************************/
#define write     _usart1_send_s
#define terminal  printf


/*******************************************************************************
 * USART1 Functions
 *******************************************************************************/
void	USART1_Config	  (void);
int		_usart1_read		(void);
int     _usart1_send_b	(int ch);
void	_usart1_send_s	(const char Message[]);

/*******************************************************************************
 * USART3 Functions
 *******************************************************************************/
void	USART3_Config   (void);
void	_usart3_send_b  (int ch);
void	_usart3_send_s  (const char Message[]);


#endif
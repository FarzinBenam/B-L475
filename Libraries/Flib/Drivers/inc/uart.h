#ifndef _UART_H
#define _UART_H

#ifdef __cplusplus
 extern "C" {
#endif



#include "main.h"

/*******************************************************************************
 * defintions
 *******************************************************************************/
#define terminal    printf





/* Exported functions --------------------------------------------------------*/ 
void  USART_Config  (USART_TypeDef *USARTx, uint32_t Baudrate);
int		_usart_read   (USART_TypeDef *USARTx);
int   _usart_send_b (USART_TypeDef *USARTx, int ch);
void	_usart_send_s	(USART_TypeDef *USARTx,const char Message[]);


#endif


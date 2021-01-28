
#include "uart.h"


void	USART1_Config (void)
{
  /* USARTx configured as follow:
        - BaudRate = USART1_BAUDRATE baud
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
	USART1->BRR		= (F_CPU / USART1_BAUDRATE);	// 115200 Bps Baud Rate at 8Mhz
	USART1->CR1		|= (1 << 3)|(1 << 2);				// RE | TE enbale
	USART1->CR1		|= (1 << 0);								// IO_0 UE: USART enable
}
void	USART3_Config (void)
{
	USART3->BRR		= (F_CPU / USART3_BAUDRATE);	// 115200 Bps Baud Rate at 8Mhz
	USART3->CR1		|= (1 << 3)|(1 << 2);				// RE | TE enbale
	USART3->CR1		|= (1 << 0);								// IO_0 UE: USART enable
}

void	_usart1_send_b			(int ch) 
{
	while(!(USART1->ISR & IO_7));					// TXE: Transmit data register empty
	
	USART1->TDR = ch;//(ch & (uint16_t)0x01FF);
}

void	_usart1_send_s			(const char Message[])
{
	volatile int i = 0;
	
	while(Message[i]){
		while(!(USART1->ISR & IO_7));
		USART1->TDR = Message[i++];
	}
}

void	_usart1_printf				(const char *format, ...)
{
	static  uint8_t  buffer[40 + 1];
	va_list     vArgs;

	va_start(vArgs, format);
	vsprintf((char *)buffer, (char const *)format, vArgs);
	va_end(vArgs);
	
	_usart1_send_s((char *) buffer);
	
	
}
int		read				(void)
{
	while(!(USART1->ISR & IO_5));					// RXNE: Read data register not empty
	
	return USART1->RDR;
}

void	_usart3_send_b			(int ch) 
{
	while(!(USART3->ISR & IO_7));					// TXE: Transmit data register empty
	
	USART3->TDR = ch;//(ch & (uint16_t)0x01FF);
}

void	_usart3_send_s			(const char Message[])
{
	volatile int i = 0;
	
	while(Message[i]){
		while(!(USART3->ISR & IO_7));
		USART3->TDR = Message[i++];
	}
}


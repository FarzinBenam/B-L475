/**
  
  * @file			
  * @author		Farzin M.Benam
  * @version	
  * @date		2020-12-18
  * @brief		
  *             uart
  *******************************************************************************
 The serial interface USART1 is directly available as a Virtual COM port of the PC 
 connected to the ST-LINK/V2-1 USB connector CN7. The Virtual COM port settings 
 are configured as: 115200 b/s, 8 bits data, no parity, 1 stop bit, no flow control.
 
 USART1_TX	PB6	AF7(as alternate fucntion AF7)
 USART1_RX	PB7	AF7(as alternate fucntion AF7)
 USR_BUTTON	PC13
  *******************************************************************************/ 
#include "uart.h"


void	USART1_Config   (void)
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
void	USART3_Config   (void)
{
	USART3->BRR		= (F_CPU / USART3_BAUDRATE);	// 115200 Bps Baud Rate at 8Mhz
	USART3->CR1		|= (1 << 3)|(1 << 2);				// RE | TE enbale
	USART3->CR1		|= (1 << 0);								// IO_0 UE: USART enable
}

int		_usart1_read            (void)
{
	while(!(USART1->ISR & IO_5));					// RXNE: Read data register not empty
	
	return USART1->RDR;
}
void	_usart1_send_b	(int ch) 
{
	while(!(USART1->ISR & IO_7));					// TXE: Transmit data register empty
	
	USART1->TDR = ch;//(ch & (uint16_t)0x01FF);
}
void	_usart1_send_s  (const char Message[])
{
	volatile int i = 0;
	
	while(Message[i]){
		while(!(USART1->ISR & IO_7));
		USART1->TDR = Message[i++];
	}
}
void	_usart1_printf  (const char *format, ...)
{
	static  uint8_t  buffer[40 + 1];
	va_list     vArgs;

	va_start(vArgs, format);
	vsprintf((char *)buffer, (char const *)format, vArgs);
	va_end(vArgs);
	
	_usart1_send_s((char *) buffer);
	
	
}


void	_usart3_send_b	(int ch) 
{
	while(!(USART3->ISR & IO_7));					// TXE: Transmit data register empty
	
	USART3->TDR = ch;//(ch & (uint16_t)0x01FF);
}

void	_usart3_send_s	(const char Message[])
{
	volatile int i = 0;
	
	while(Message[i]){
		while(!(USART3->ISR & IO_7));
		USART3->TDR = Message[i++];
	}
}


/******************************************************************************* 
 * interface to the stdio.h library.
 * All the I/O directed to the console.
********************************************************************************/
struct	__FILE {
	int handle;
	 /* Whatever you require here. If the only file you are using is
		* standard output using printf() for debugging, no file handling
		* is required. */
};

FILE __stdin = {0};
FILE __stdout = {1};
FILE __stderr = {2};

int fgetc (FILE *f){
    int c;
    c = _uart1_read();           // read the character from console
    
    if (c == '\r'){             // if \r, after it is echoed, a \n sent
        _usart1_write(c);         // echo
        c = '\n';
    }
    uart1_write(c);            // echo
    
    return c;
}

int fputc (int c, FILE *f){
    return uart1_write(c);      // write the character to console
}
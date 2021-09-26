/**
  
  * @file			
  * @author		Farzin M.Benam
  * @version	
  * @date     2020-12-18
  * @brief		
  *             
  *******************************************************************************
 The serial interface USART1 is directly available as a Virtual COM port of the PC 
 connected to the ST-LINK/V2-1 USB connector CN7. The Virtual COM port settings 
 are configured as: 115200 b/s, 8 bits data, no parity, 1 stop bit, no flow control.
 
 USART1_TX	PB6	AF7(as alternate fucntion AF7)
 USART1_RX	PB7	AF7(as alternate fucntion AF7)
 
 UART4_TX	PA0	AF8(as alternate fucntion AF8)
 UART4_RX	PA1	AF8(as alternate fucntion AF8)
 
 USR_BUTTON	PC13
  *******************************************************************************/ 
#include "uart.h"


/*******************************************************************************
 * USART Config Functions
 *******************************************************************************/
void USART_Config       (USART_TypeDef *USARTx, uint32_t Baudrate)
{
	if(USARTx == USART1)
	{
			// Related Peripherals Clock Enable
			/*******************************************************************************/  
			SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIOBEN);	    // Bit 1 GPIOBEN: IO port B clock enable
			SET_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN);    // Bit 14 USART1EN: USART1clock enable
			
			// USART1 RX-TX PINs GPIO init (PB6=TX, PB7=RX) (over STLINK)
			/*******************************************************************************/  
			// GPIO alternate function  (AF7 = 0b0111)
			SET_BIT(GPIOB->AFR[0],( GPIO_AFRL_AFSEL6|GPIO_AFRL_AFSEL7));        // set all to 0x1111
			CLEAR_BIT(GPIOB->AFR[0],(GPIO_AFRL_AFSEL6_3|GPIO_AFRL_AFSEL7_3));   // clear last bit = 0x0111
			
			// GPIO port mode register set to Alternate function mode
			SET_BIT(GPIOB->MODER, (GPIO_MODER_MODE6_1|GPIO_MODER_MODE7_1));   
			CLEAR_BIT(GPIOB->MODER, (GPIO_MODER_MODE6_0|GPIO_MODER_MODE7_0));
	}
	else if (USARTx == USART3)
	{
			// Related Peripherals Clock Enable
			/*******************************************************************************/  
			SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIODEN);     // Bit 3 GPIODEN: IO port D clock enable
			SET_BIT(RCC->APB1ENR1, RCC_APB1ENR1_USART3EN);  // Bit 18 USART3EN: USART3 clock enable
			
			// USART3 RX-TX PINs GPIO init (PD8=TX, PD9=RX) (over ISM43362-M3G-L44)
			/*******************************************************************************/  
			// GPIO alternate function  (AF7 = 0b0111)
			SET_BIT(GPIOD->AFR[1],( GPIO_AFRH_AFSEL8|GPIO_AFRH_AFSEL9));        // set all to 0b1111
			CLEAR_BIT(GPIOD->AFR[1],(GPIO_AFRH_AFSEL8_3|GPIO_AFRH_AFSEL9_3));   // clear last bit = 0b0111
			
			// GPIO port mode register set to Alternate function mode
			SET_BIT(GPIOD->MODER, (GPIO_MODER_MODE8_1|GPIO_MODER_MODE9_1));   
			CLEAR_BIT(GPIOD->MODER, (GPIO_MODER_MODE8_0|GPIO_MODER_MODE9_0));
	}
	else if (USARTx == UART4)
	{

			// Related Peripherals Clock Enable
			/*******************************************************************************/  
			SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIOAEN);	    // Bit 0 GPIOAEN: IO port A clock enable
			SET_BIT(RCC->APB1ENR1, RCC_APB1ENR1_UART4EN);   // Bit 19 UART4EN: uart4 clock enable
			
			// USART4 RX-TX PINs GPIO init (PA0, PA1) (over HC-05)
			/*******************************************************************************/  
			// GPIO alternate function  (AF8 = 0b1000)
			CLEAR_BIT(GPIOA->AFR[0], (GPIO_AFRL_AFSEL0|GPIO_AFRL_AFSEL1));      // clear the bits 0b0000
			SET_BIT(GPIOA->AFR[0],(GPIO_AFRL_AFSEL0_3|GPIO_AFRL_AFSEL1_3));     // set the last bit = 0b1000

			
			// GPIO port mode register set to Alternate function mode
			SET_BIT(GPIOA->MODER, (GPIO_MODER_MODE0_1|GPIO_MODER_MODE1_1));   
			CLEAR_BIT(GPIOA->MODER, (GPIO_MODER_MODE0_0|GPIO_MODER_MODE1_0));
	}

	
//	USARTx->BRR = (F_CPU / Baudrate);	            // xxxx Bps Baud Rate at xx Mhz
	USARTx->CR1	|= (USART_CR1_RE | USART_CR1_TE);	// RE | TE enbale
	USARTx->CR1	|= USART_CR1_UE;					// IO_0 UE: USART enable
     
}

int		_usart_read            (USART_TypeDef *USARTx)
{
	while(!(USARTx->ISR & USART_ISR_RXNE));					// RXNE: Read data register not empty
	
	return USARTx->RDR;
}

int   _usart_send_b	(USART_TypeDef *USARTx, int ch) 
{    
	while(!(USARTx->ISR & USART_ISR_TXE));					// TXE: Transmit data register empty
	USARTx->TDR = (ch & (uint16_t)0x01FF);

    return ch;
}
void	_usart_send_s  (USART_TypeDef *USARTx, const char Message[])
{
	volatile int i = 0;

	while(Message[i]){
		while(!(USARTx->ISR & USART_ISR_TXE));
		USARTx->TDR = Message[i++];
	}
}





/******************************************************************************* 
 * interface to the stdio.h library.
 * All the I/O directed to the console.
 * after this we can use the stdio functions like: printf
********************************************************************************/
//to avoid redifintion of the __FILE we comment this session
//struct	__FILE {
//	int handle;
//	 /* Whatever you require here. If the only file you are using is
//		* standard output using printf() for debugging, no file handling
//		* is required. */
//};

FILE __stdin =  {0};
FILE __stdout = {1};
FILE __stderr = {2};

int fgetc (FILE *f){
    int c;
    c = _usart_read(terminal_usart);            // read the character from console
    
    if (c == '\r'){                             // if \r, after it is echoed, a \n sent
        _usart_send_b(terminal_usart, c);       // echo
        c = '\n';
    }
    _usart_send_b(terminal_usart, c);           // echo
    
    return c;
}

int fputc (int c, FILE *f){
  return _usart_send_b(terminal_usart, c);    // write the character to console
}




 /******************* (C) COPYRIGHT 2021 Farzin_M.Benam *****END OF FILE****/

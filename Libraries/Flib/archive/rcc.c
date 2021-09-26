#include "rcc.h"









#if defined (USE_HAL_DRIVER)
 
 
#else

void	RCC_Config		(void)
{
	RCC->APB1ENR1 |= (1 << 10); 			// Bit 10 RTCAPBEN: RTC APB clock enable
	RCC->APB1ENR1 |= (1 << 28);				// Bit 28 PWREN: Power interface clock enable
	RCC->APB1ENR1 |= (1 << 15);			// Bit 15 SPI3EN: SPI3 clock enable
	RCC->APB1ENR1 |= (1 << 18);				// Bit 18 USART3EN: USART3 clock enable
    RCC->APB1ENR1 |= (1 << 22);             // Bit 22 I2CEN: i2c clock enable
    RCC->APB1ENR1 |= (1 << 19);           // Bit 19 UART4EN: uart4 clock enable
	
	// this bit is enabled in ISR_Config() for better understanding
//	RCC->APB2ENR	|= RCC_APB2ENR_SYSCFGEN;// Bit 0 SYSCFGEN: SYSCFG + COMP + VREFBUF clock enable
	RCC->APB2ENR	|= RCC_APB2ENR_USART1EN;// Bit 14 USART1EN: USART1clock enable
	
    /* GPIO Ports Clock Enable */
	RCC->AHB2ENR	|= RCC_AHB2ENR_GPIOAEN;	// Bit 0 GPIOAEN: IO port A clock enable
	RCC->AHB2ENR	|= RCC_AHB2ENR_GPIOBEN;	// Bit 1 GPIOBEN: IO port B clock enable
	RCC->AHB2ENR	|= RCC_AHB2ENR_GPIOCEN;	// Bit 2 GPIOCEN: IO port C clock enable
	RCC->AHB2ENR	|= RCC_AHB2ENR_GPIODEN;	// Bit 3 GPIODEN: IO port D clock enable
	RCC->AHB2ENR	|= RCC_AHB2ENR_GPIOEEN;	// Bit 4 GPIOEEN: IO port E clock enable
    
}


#endif /* USE_HAL_DRIVER */


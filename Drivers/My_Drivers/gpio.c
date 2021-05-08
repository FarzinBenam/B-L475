#include "Gpio.h"


void	GPIO_Config		(void)
{
	LEDs_Init();
	
	
	// Push Button init
	/*******************************************************************************/  
	GPIOC->MODER &= ~(IO_26|IO_27);			// set PC13 input
	
	
	// USART1 RX-TX Pins init
	/*******************************************************************************/  
	GPIOB->AFR[0]	|= (1 << 24)|(1 << 25)|(1 << 26)|(1 << 28)|(1 << 29)|(1 << 30);
	GPIOB->MODER 	|= (1 << 13)|(1 << 15); // Alternate function mode
	GPIOB->MODER 	&= ~(IO_12 | IO_14);		// Alternate function mode
	
	// USART3 RX-TX Pins init (to communicate with ISM43362-M3G-L44)
	/*******************************************************************************/  
	GPIOD->AFR[1]	|= (1 << 0)|(1 << 1)|(1 << 2)|(1 << 4)|(1 << 5)|(1 << 6);
	GPIOD->MODER 	|= (1 << 17)|(1 << 19); // Alternate function mode
	GPIOD->MODER 	&= ~(IO_16 | IO_18);		// Alternate function mode
	
	// RTC
	/*******************************************************************************/  
	
	
//	// SPI_ISM43362-M3G-L44
//	/*******************************************************************************/
//	GPIOB->MODER	|= IO_26;								// PB13	ISM43362-WAKEUP output
//	GPIOB->MODER	&= ~IO_26;							// PB13	ISM43362-WAKEUP output
//	
//	GPIOC->AFR[1]	|= (IO_9|IO_10|IO_13|IO_14|IO_17|IO_18);	// CLK, MOSI, MISO Alternate function mode
//	GPIOC->AFR[1]	&= ~(IO_8|IO_11|IO_12|IO_15|IO_16|IO_19);	// CLK, MOSI, MISO Alternate function mode
//	GPIOC->MODER 	|= (IO_21|IO_23|IO_25);										// Alternate function mode
//	GPIOC->MODER 	&= ~(IO_20|IO_22|IO_24);									// Alternate function mode
//	GPIOC->OSPEEDR|= (IO_20|IO_22|IO_24);										// CLK, MOSI, MISO Medium speed
//	GPIOC->OSPEEDR &= ~(IO_21|IO_23|IO_25);									// CLK, MOSI, MISO Medium speed
//	
//	GPIOE->MODER	|= IO_0;								// PE0	ISM43362-SPI3_CSN		output(SPI NSS pin)
//	GPIOE->MODER 	&= ~(IO_1|IO_2|IO_3);		// PE1	ISM43362-DRDY_EXTI1 input	(Data ready pin)
//	GPIOE->OSPEEDR|= IO_0;								// NSS pin Medium speed
//	GPIOE->OSPEEDR&= ~IO_1;								// NSS pin Medium speed
//	GPIOE->MODER	|= IO_16;								// PE8	ISM43362-RST 				output(Reset pin)
//	GPIOE->MODER	&= ~IO_17;							// PE8	ISM43362-RST 				output(Reset pin)
//  
//  
//  // I2C_2
//	/*******************************************************************************/
//  // PB10 = I2C2_SCL
//  // PB11 = I2C2_SDA
//  
//  GPIOB->MODER |= (IO_21|IO_23);                // Alternate function mode
//  GPIOB->MODER &= ~(IO_20|IO_22);               // Alternate function mode
//  
//  GPIOB->OSPEEDR |= (IO_20|IO_21|IO_22|IO_23);  // Very high speed mode
//  
//  GPIOB->AFR[1] |= (IO_10|IO_14);               // I2C2_SCL & I2C2_SDA
//  GPIOB->AFR[1] &= ~(IO_8|IO_9|IO_11|IO_12|IO_13|IO_15);
  
}

void    LEDs_Init         (void){
    // LED_init
	/*******************************************************************************/ 
	GPIOA->MODER |= IO_10;					// set PA5 output 
	GPIOA->MODER &= ~(IO_11);           	// set PA5 output 
	GPIOB->MODER |= IO_28;              	// set PB14 output 
	GPIOB->MODER &= ~(IO_29);           	// set PB14 output 
	GPIOC->MODER |= IO_8;					// set PC4 output 
	GPIOC->MODER &= ~(IO_9);          	 	// set PC4 output
	GPIOC->MODER |= IO_10;					// set PC5 output 
	GPIOC->MODER &= ~(IO_11);          	 	// set PC5 output
	
	GPIOC->MODER |= IO_18;					// set PC9 output (WiFi & Bluetooth)
	GPIOC->MODER &= ~(IO_19);          	 	// set PC9 output
    
}



#ifndef MAIN_H_
#define MAIN_H_

#include <stdio.h>
#include <stdlib.h>
#include "stm32l4xx.h"                  // Device header

#include "Drivers/uart.h"
#include "Drivers/gpio.h"
#include "Drivers/i2c.h"
#include "Configs.h"

///////////////////////////////////
// Configurations
#define rtos            0
#define COM             0               // usart1 used to communicate with COM port
#define hc_05           1               // uart4 used if the hc-05 ble module used


#define F_CPU       	SystemCoreClock	// 4MHz processor
#define terminal_hc_05  hc_05
#define terminal_COM    !terminal_hc_05



//////////////////////////////////////////////////////////////////////
#if (COM == 0) && (hc_05 == 0)
#error	"No Terminal available! Neither COM port nor HC-05 are not defined!"
#endif

#if (terminal_hc_05 == 1)

#define terminal_usart  UART4
#define terminal_IRQn   UART4_IRQn

#elif (COM == 1)

#define terminal_usart  USART1
#define terminal_IRQn   USART1_IRQn

#endif

#if (hc_05 == 1)

#define hc_05_usart      UART4
#define hc_05_baudrate   UART4_BAUDRATE
#define hc_05_IRQn       UART4_IRQn

#endif

#if (COM == 1)

#define COM_usart      USART1
#define COM_baudrate   USART1_BAUDRATE
#define COM_IRQn       USART1_IRQn

#endif


#define USART1_BAUDRATE	115200
#define USART3_BAUDRATE	115200
#define UART4_BAUDRATE	9600


#ifndef F_CPU
#error "F_CPU must be defined for this to work"
#endif

#ifndef USART1_BAUDRATE
#error "USART1_BAUDRATE1 must be defined for usart to work"
#endif
#ifndef USART3_BAUDRATE
#error "USART3_BAUDRATE1 must be defined for usart to work"
#endif


//************************************************************************
//* Specific Definitions 
//************************************************************************
#define MAX_COMMANDS        20
#define CMD_BUFFER_SIZE     32
#define IIC_BUFFER_SIZE     512
#define	WIFI_BUFFER_SIZE    1024


typedef struct {
	uint16_t year;		/* 1..4095 */
	uint8_t month;		/* 1..12 */
	uint8_t mday;		/* 1..31 day of the month (date) */
	uint8_t wday;		/* 0..6, Sunday = 0 day of the week (day) */
	uint8_t ampm;		/* 12h or 24h format */
	uint8_t hour;		/* 0..23 */
	uint8_t min;		/* 0..59 */
	uint8_t sec;		/* 0..59 */
	uint8_t ssec;		/* sub-seconds (not programmable) */
}RTC_t;
/************************************************************************
 * Macros
 ************************************************************************/
/**
 * determine size (number of elements) in an array
 */
#define ARRAY_SIZE(a)   ( sizeof(a) / sizeof(a[0]) )

/**
 * Macro to determine the element index in an array from the element address
 */
#define ARRAY_POSITION( array, element_pointer )     ( ((uint32_t)element_pointer - (uint32_t)array) / sizeof(array[0]) )

#define LED1_ON()		SET_BIT(LED1_PORT->BSRR, LED1_PIN)
#define LED1_OFF()	SET_BIT(LED1_PORT->BRR, LED1_PIN)

#define LED2_ON()		SET_BIT(LED2_PORT->BSRR, LED2_PIN)
#define LED2_OFF()	SET_BIT(LED2_PORT->BRR, LED2_PIN)

#define LED3_ON()		SET_BIT(LED3_PORT->BSRR, LED3_PIN)
#define LED3_OFF()	SET_BIT(LED3_PORT->BRR, LED3_PIN)





#define SPI_IS_BUSY(SPIx)			(((SPIx)->SR & (SPI_SR_TXE | SPI_SR_RXNE)) == 0 || ((SPIx)->SR & SPI_SR_BSY))

#define SPI_WAIT(SPIx)				while (SPI_IS_BUSY(SPIx)) {}

#define SPI_CHECK_ENABLED(SPIx)     if (!((SPIx)->CR1 & SPI_CR1_SPE)) {goto error;}

#define SPI3_EN()					SPI3->CR1 |= SPI_CR1_SPE;

#define SPI3_DIS()					SPI3->CR1 &= ~SPI_CR1_SPE;

#define NSS_EN()					GPIOE->BRR |= NSS;

#define NSS_DIS()					GPIOE->BSRR |= NSS;

#define WIFI_RST()					GPIOE->BRR |= WIFI_RST_PIN;\
                    delayMs(100);\
									GPIOE->BSRR |= WIFI_RST_PIN;\
									delayMs(500);\





#define	OneSec          F_CPU		// for use in SysTick
#define	OneMilliSec     4000		// for use in SysTick
//************************************************************************
//*	Pin-Port Definitions
//************************************************************************
// LEDs
#define LED1_PIN_Pos    (5U)
#define LED1_PIN_Msk    (0x1UL << LED1_PIN_Pos)
#define LED1_PIN        LED1_PIN_Msk
#define LED1_PORT       GPIOA

#define LED2_PIN_Pos    (14U)
#define LED2_PIN_Msk    (0x1UL << LED2_PIN_Pos)
#define LED2_PIN        LED2_PIN_Msk
#define LED2_PORT       GPIOB

#define LED3_PIN_Pos    (9U)
#define LED3_PIN_Msk    (0x1UL << LED3_PIN_Pos)
#define LED3_PIN        LED3_PIN_Msk
#define LED3_PORT       GPIOC
//******************************************
// Push-Button
#define PB1_PIN         IO_13
#define PB1_PORT        IO_13

//******************************************
// WiFi (ISM43362)
#define WIFI_RST_PIN    GPIO_PIN_8
#define WIFI_RST_PORT   GPIOE

#define WIFI_BOOT0_PIN      GPIO_PIN_12
#define WIFI_BOOT0_PORT     GPIOB

#define WIFI_WAKEUP_PIN     GPIO_PIN_13
#define WIFI_WAKEUP_PORT    GPIOB

#define WIFI_RDY_PIN        GPIO_PIN_1
#define WIFI_RDY_PORT       GPIOE
#define WIFI_RDY_EXTI_IRQn  EXTI1_IRQn

#define WIFI_CSN_PIN        GPIO_PIN_0 // NSS pin
#define WIFI_CSN_PORT       GPIOE

//******************************************
// SPI3
#define SPI3_SCK_PIN    GPIO_PIN_10
#define SPI3_SCK_PORT   GPIOC

#define SPI3_MISO_PIN   GPIO_PIN_11
#define SPI3_MISO_PORT  GPIOC

#define SPI3_MOSI_PIN   GPIO_PIN_12
#define SPI3_MOSI_PORT  GPIOC




#define ENTER         0x0D
#define SPACE         ' '
#define STX           0x02
#define BACKSPACE1    0x08
#define BACKSPACE2    0x7F
#define CommandSign   '>'
#define STAR          '*'
#define UNDERLINE     '_'
#define SCAPE_BUT     '`'



#endif /* MAIN_H_ */



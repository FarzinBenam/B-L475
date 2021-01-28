#ifndef FCMD_H_
#define FCMD_H_

#include <stdint.h>
///////////////////////////////////
#define USART1_BAUDRATE	115200//115200
#define USART3_BAUDRATE	115200

#define F_CPU       		SystemCoreClock		// 4MHz processor

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
#define MAX_COMMANDS		20
#define CMD_BUFFER_SIZE	32
#define IIC_BUFFER_SIZE	512
#define	WIFI_BUFFER_SIZE 1024


//************************************************************************
//* Specific Structure Definitions
//************************************************************************/
typedef struct{
	uint8_t	ProgramStatus :1;
	//CmdStatus = 0 -> normal status
	//CmdStatus = 1 -> command Entered
	//CmdStatus = 2 -> cmd without parameter
	//CmdStatus = 3 -> cmd with parameter
	//CmdStatus = 4 -> Usart RX intterupt
	//
	//
	//
	uint8_t	CmdStatus :3;
	uint8_t	CmdProcessStatus :1; 
	uint8_t	System1SecStatus1 :1;
	uint8_t	System1SecStatus2 :1;
	uint8_t	Unused:1;
}Status;


typedef struct{
	// Pointer to a CMD
	const char *Command;
	// Pointer to Function
	void (*func_ptr)(void);
} Commands;

typedef struct {
	uint16_t year;		/* 1..4095 */
	uint8_t month;		/* 1..12 */
	uint8_t mday;			/* 1..31 day of the month (date) */
	uint8_t wday;			/* 0..6, Sunday = 0 day of the week (day) */
	uint8_t ampm;			/* 12h or 24h format */
	uint8_t hour;			/* 0..23 */
	uint8_t min;			/* 0..59 */
	uint8_t sec;			/* 0..59 */
	uint8_t ssec;			/* sub-seconds (not programmable) */
}RTC_t;
/************************************************************************
 * Macros
 ************************************************************************/
/**
 * determine size (number of elements) in an array
 */
#define ARRAY_SIZE(a)                                ( sizeof(a) / sizeof(a[0]) )

/**
 * Macro to determine the element index in an array from the element address
 */
#define ARRAY_POSITION( array, element_pointer )     ( ((uint32_t)element_pointer - (uint32_t)array) / sizeof(array[0]) )

#define LED1_ON()		GPIOA->BSRR |= LD1;//\
										GPIOA->BSRR &= ~(LD1 << 16U)
#define LED1_OFF()	GPIOA->BRR |= LD1

#define LED2_ON()		GPIOB->BSRR |= LD2;//\
										GPIOB->BSRR &= ~(LD2 << 16U)
#define LED2_OFF()	GPIOB->BRR |= LD2

#define LED3_ON()		GPIOC->BSRR |= LD3;//\
										GPIOB->BSRR &= ~(LD2 << 16U)
#define LED3_OFF()	GPIOC->BRR |= LD3




#define SPI_IS_BUSY(SPIx)					(((SPIx)->SR & (SPI_SR_TXE | SPI_SR_RXNE)) == 0 || ((SPIx)->SR & SPI_SR_BSY))

#define SPI_WAIT(SPIx)						while (SPI_IS_BUSY(SPIx)) {}

#define SPI_CHECK_ENABLED(SPIx)		if (!((SPIx)->CR1 & SPI_CR1_SPE)) {goto error;}

#define SPI3_EN()									SPI3->CR1 |= SPI_CR1_SPE;

#define SPI3_DIS()								SPI3->CR1 &= ~SPI_CR1_SPE;

#define NSS_EN()									GPIOE->BRR |= NSS;

#define NSS_DIS()									GPIOE->BSRR |= NSS;

#define WIFI_RST()								GPIOE->BRR |= WIFI_RST_PIN;\
																	delayMs(100);\
																	GPIOE->BSRR |= WIFI_RST_PIN;\
																	delayMs(500);\





#define IO_0    ((uint32_t)0x00000001)		  /*!< Pin 0 selected */
#define IO_1    ((uint32_t)0x00000002)		  /*!< Pin 1 selected */
#define IO_2    ((uint32_t)0x00000004)		  /*!< Pin 2 selected */
#define IO_3    ((uint32_t)0x00000008)		  /*!< Pin 3 selected */
#define IO_4    ((uint32_t)0x00000010)		  /*!< Pin 4 selected */
#define IO_5    ((uint32_t)0x00000020)		  /*!< Pin 5 selected */
#define IO_6    ((uint32_t)0x00000040)		  /*!< Pin 6 selected */
#define IO_7    ((uint32_t)0x00000080)		  /*!< Pin 7 selected */
#define IO_8    ((uint32_t)0x00000100)		  /*!< Pin 8 selected */
#define IO_9    ((uint32_t)0x00000200)		  /*!< Pin 9 selected */
#define IO_10   ((uint32_t)0x00000400)		  /*!< Pin 10 selected */
#define IO_11   ((uint32_t)0x00000800)		  /*!< Pin 11 selected */
#define IO_12   ((uint32_t)0x00001000)		  /*!< Pin 12 selected */
#define IO_13   ((uint32_t)0x00002000)		  /*!< Pin 13 selected */
#define IO_14   ((uint32_t)0x00004000)		  /*!< Pin 14 selected */
#define IO_15   ((uint32_t)0x00008000)		  /*!< Pin 15 selected */
#define IO_16   ((uint32_t)0x00010000)		  /*!< Pin 16 selected */
#define IO_17   ((uint32_t)0x00020000)		  /*!< Pin 17 selected */
#define IO_18   ((uint32_t)0x00040000)		  /*!< Pin 18 selected */
#define IO_19   ((uint32_t)0x00080000)		  /*!< Pin 19 selected */
#define IO_20   ((uint32_t)0x00100000)		  /*!< Pin 20 selected */
#define IO_21   ((uint32_t)0x00200000)		  /*!< Pin 21 selected */
#define IO_22   ((uint32_t)0x00400000)		  /*!< Pin 22 selected */
#define IO_23   ((uint32_t)0x00800000)		  /*!< Pin 23 selected */
#define IO_24   ((uint32_t)0x01000000)		  /*!< Pin 24 selected */
#define IO_25   ((uint32_t)0x02000000)		  /*!< Pin 25 selected */
#define IO_26   ((uint32_t)0x04000000)		  /*!< Pin 26 selected */
#define IO_27   ((uint32_t)0x08000000)		  /*!< Pin 27 selected */
#define IO_28   ((uint32_t)0x10000000)		  /*!< Pin 28 selected */
#define IO_29   ((uint32_t)0x20000000)		  /*!< Pin 29 selected */
#define IO_30   ((uint32_t)0x40000000)		  /*!< Pin 30 selected */
#define IO_31   ((uint32_t)0x80000000)		  /*!< Pin 31 selected */


#define	OneSec        F_CPU		// for use in SysTick
#define	OneMilliSec   4000		// for use in SysTick

#define LD1           IO_5
#define LD2           IO_14
#define LD3           IO_9
#define PB1           IO_13
#define NSS           IO_0
#define WIFI_RDY_PIN  IO_1
#define WIFI_RST_PIN	IO_8



#define ENTER         0x0D
#define SPACE         ' '
#define STX           0x02
#define BACKSPACE1    0x08
#define BACKSPACE2    0x7F
#define CommandSign   '>'
#define STAR          '*'
#define UNDERLINE     '_'

#endif /* FCMD_H_ */



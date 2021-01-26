/*******************************************************************************/ 
USART1:


The serial interface USART1 is directly available as a Virtual COM port of the PC 
connected to the ST-LINK/V2-1 USB connector CN7. The Virtual COM port settings 
are configured as: 115200 b/s, 8 bits data, no parity, 1 stop bit, no flow control.

USART1_TX	PB6	AF7(as alternate fucntion AF7)
USART1_RX	PB7	AF7(as alternate fucntion AF7)

always use a Local Echo OFF Terminal as an serial interface
/*******************************************************************************/ 
 
 USR_BUTTON	PC13
 
/*******************************************************************************/ 
SPI (ISM43362-M3G-L44):
Clock rate: 20MHz max, Width: 16-bit, Mode: 0, Endian: Little

PB12	ISM43362-BOOT0
PB13	ISM43362-WAKEUP

PC10	INTERNAL-SPI3_SCK
PC11	INTERNAL-SPI3_MISO
PC12	INTERNAL-SPI3_MOSI

PE0		ISM43362-SPI3_CSN
PE1		ISM43362-DRDY_EXTI1
PE8		ISM43362-RST

Mode              = SPI_MODE_MASTER;
Direction         = SPI_DIRECTION_2LINES;
DataSize          = SPI_DATASIZE_16BIT;
CLKPolarity       = SPI_POLARITY_LOW;
CLKPhase          = SPI_PHASE_1EDGE;
NSS               = SPI_NSS_SOFT;
BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; /* 80/8= 10MHz (Inventek WIFI module supportes up to 20MHz)*/
FirstBit          = SPI_FIRSTBIT_MSB;
TIMode            = SPI_TIMODE_DISABLE;
CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
/*******************************************************************************/ 

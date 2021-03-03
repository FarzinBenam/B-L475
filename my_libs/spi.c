#include "spi.h"

//************************************************************************
//*	Definitions
//************************************************************************
extern volatile int tick;
extern volatile int _tick;

extern volatile uint8_t	CmdStatus;




void	SPI3_Config 	(void)
{
	/*
	Mode              = SPI_MODE_MASTER;
	Direction         = SPI_DIRECTION_2LINES;
	DataSize          = SPI_DATASIZE_16BIT;
	CLKPolarity       = SPI_POLARITY_LOW;
	CLKPhase          = SPI_PHASE_1EDGE;
	NSS               = SPI_NSS_SOFT;
	BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; // 80/8= 10MHz (Inventek WIFI module supportes up to 20MHz)
	FirstBit          = SPI_FIRSTBIT_MSB;
	TIMode            = SPI_TIMODE_DISABLE;
	CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
	*/
	SPI3_DIS();														// Make sure that the peripheral is off, and reset it.
	
	SPI3->CR1 &= ~IO_0;										// clock polarity and phase. (Bit 0 CPHA: Clock phase)
	SPI3->CR1 &= ~IO_1;										// clock polarity and phase. (Bit1 CPOL: Clock polarity)
	SPI3->CR1 |= (1 << 2);								// Bit 2 MSTR: Master selection (STM32 as the host device)
	SPI3->CR1 |= (1 << 8);								// Bit 8 SSI: Internal slave select. (Set software 'Chip Select' pin.)
	SPI3->CR1 |= (1 << 9);								// Bit 9 SSM: Software slave management. (Set the internal 'Chip Select' signal.)
	SPI3->CR1 &= ~IO_13;									// Bit 13 CRCEN: Hardware CRC calculation enable (CRC_DISABLE)
	
	SPI3->CR2 |= (IO_0|IO_1|IO_2|IO_3);		// Bits 11:8 DS [3:0]: Data size (1111: 16-bit)
	SPI3->CR2 &= ~IO_4;										// Bit 4 FRF: Frame format (TIMODE_DISABLE)
	
	SPI3_EN();														// Bit 6 SPE: SPI enable
}
/**
  * @brief  Transmit an amount of data in non-blocking mode with Interrupt.
  * @param  pData: pointer to data buffer
  * @param  Size: amount of data to be sent
  */
int		SPI_Transmit	(uint8_t *pData, uint16_t Size)
{
	int TxCount = Size;
	int TimeoutCount = 0;
	uint16_t buff;
	
		
	// check for null data or size
	if ((pData == NULL) || (Size == 0U))	goto error;
	
	// Check if the SPI is already enabled
	SPI_CHECK_ENABLED(SPI3);
	
	
//	nl(1);
	// Transmit data in 16 Bit mode
	while (TxCount > 0U){
		buff = *pData;
		buff = (buff << 8);
		++pData;
		buff |= (*pData & 0x00FF);
		++pData;
		
		// Wait until TXE flag is set to send data
		if ((SPI3->SR & SPI_SR_TXE) == SPI_SR_TXE){
//			_usart1_printf("%.4X - ", buff);
			SPI3->DR = buff;
			TxCount--;
			delayMs(1);
			buff = SPI3->DR;
//			_usart1_printf("%.4X\n\r", buff);
		}
		else
		{
			TimeoutCount++;
			// Timeout management
			if (TimeoutCount > 20)	goto error;
		}
	}

//	_usart1_printf("SPI Send OK!");
	return 0;
	
	error :
		nl(1);
		terminal("SPI Transmit Error");
		return -1;	
}
/**
  * @brief  Receive an amount of data in blocking mode.
  * @param  pData: pointer to data buffer
  * @param  Size: amount of data to be received
  */
int		SPI_Receive8	(uint8_t *pData, uint16_t Size)
{
	int RxCount;
	
	// check for null data or size
  if (Size == 0U)	goto error;
	
  // Check if the SPI is already enabled
	SPI_CHECK_ENABLED(SPI3);
	
	/* Wait for previous transmissions to complete if DMA TX enabled for SPI */
	SPI_WAIT(SPI3);
	

	for (RxCount = 0; RxCount < Size; RxCount++) {
		/*	Fill output buffer with Dummy byte to be sent over SPI,
				to receive data back. In most cases 0x00 or 0xFF */
		SPI3->DR = 0x00;
		
		/* Wait for SPI to end everything */
		SPI_WAIT(SPI3);
		
		/* Save data to buffer */
		pData[RxCount] = SPI3->DR;
	}
	
	return 0;
	
error :
	nl(1);
	terminal("SPI Recieve Error");
  return -1;
}
/**
  * @brief  Receive an amount of data in blocking mode.
  * @param  pData: pointer to data buffer
  * @param  Size: amount of data to be received
  */
int		SPI_Receive16	(uint8_t *pData, int cmd)
{
	volatile int RxCount = 0;
	uint16_t temp;
	int timeout = 0;
	
//	_usart1_printf("\n\rSPI Recieve");

	
  // Check if the SPI is already enabled
	SPI_CHECK_ENABLED(SPI3);
	
	// Wait for previous transmissions to complete if DMA TX enabled for SPI
	SPI_WAIT(SPI3);
	
//	nl(1);
	//for (RxCount = 0; RxCount < Size; RxCount++) {
	while(GPIOE->IDR & WIFI_RDY_PIN){
		LED1_ON();
		
		/*	Fill output buffer with Dummy byte to be sent over SPI,
				to receive data back. In most cases 0x00 or 0xFF but here 0x0A0A*/
		SPI3->DR = cmd;
		
		// Wait for SPI to end everything
		SPI_WAIT(SPI3);
		
		// Save data to buffer
		temp = SPI3->DR;
		
		pData[RxCount] = temp;
//	_usart1_printf("%.2X-", pData[RxCount]);
		RxCount++;
		
		pData[RxCount] = (uint8_t)((temp & 0xFF00) >> 8);
//		_usart1_printf("%.2X ", pData[RxCount]);
		RxCount++;
		
		// This the last data
		if(!(GPIOE->IDR & WIFI_RDY_PIN)){
			CmdStatus = 0;
			// Save data to buffer
			temp = SPI3->DR;
//			_usart1_printf("\n\r-%.4X", temp);
			
			pData[RxCount] =  (uint8_t)(temp & 0x00FF); 
////			_usart1_printf("---%.2X-", pData[RxCount]);
			RxCount++;
			pData[RxCount] = (uint8_t)((temp & 0xFF00) >> 8);
//			_usart1_printf(" %.2X", pData[RxCount]);
			break;
			
		}
	}
	LED1_OFF();

	return RxCount;
	
error :
	nl(1);
	terminal("SPI Recieve Error");
	CmdStatus = 0;
  return -1;
}



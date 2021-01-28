#ifndef _SPI_H
#define _SPI_H


#include "stm32l4xx.h"                  // Device header
#include "fcmd.h"
#include "uart.h"
#include "Processes.h"
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>


void	SPI3_Config 			(void);
int		SPI_Transmit			(uint8_t *pData, uint16_t Size);
int		spi_wifi_send			(uint8_t *pdata,  uint16_t len);
int		spi_recieve8			(uint8_t dat);
int		SPI_Receive16			(uint8_t *pData, int cmd);

#endif
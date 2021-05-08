#ifndef _SPI_H
#define _SPI_H

#include "fcmd.h"


#ifdef __cplusplus
 extern "C" {
#endif




/**
 * @brief  Initializes the SPI for BlueNRG module
 * @param  xSpiMode : Spi mode (Polling or IRQ)
 * @retval Status
 */

//void	SPI_Config 			    (SdkEvalSpiMode xSpiMode);
uint8_t SPI_Irq_Pin             (void);
void    SPI_EXTI_Flag_Clear     ();
void    SPI_IRQ_Enable          (void);
void    SPI_IRQ_Disable         (void);


uint8_t BlueNRG_SPI_Read_All    (uint8_t *buffer, uint8_t buff_size);
int16_t BlueNRG_SPI_Write       (uint8_t* data1, uint8_t* data2, uint16_t Nb_bytes1, uint16_t Nb_bytes2);
uint8_t BlueNRG_DataPresent     (void);
void    BlueNRG_IRQ_High        (void);
void    BlueNRG_Release_IRQ     (void);
int16_t BlueNRG_SPI_Write_Bridge(uint8_t* data, uint16_t Nb_bytes);
int16_t BlueNRG_SPI_Read_Bridge (void);



void    SpiMisoPinInit          (void);
void    SpiDtmMisoPinState      (FunctionalState state);



int		SPI_Transmit			(uint8_t *pData, uint16_t Size);
int		spi_wifi_send			(uint8_t *pdata,  uint16_t len);
int		spi_recieve8			(uint8_t dat);
int		SPI_Receive16			(uint8_t *pData, int cmd);

#endif
#ifndef _CONFIGS_H
#define _CONFIGS_H

#ifdef __cplusplus
 extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Configurations    ---------------------------------------------------------*/
#define rtos            0
#define COM             1               // usart1 used to communicate with COM port
#define hc_05           0              // uart4 used if the hc-05 ble module used
#define systick         1               // use RTC for systick
#define qspi_debug      0


//#define F_CPU           SystemCoreClock
#define terminal_hc_05  hc_05
#define terminal_COM    !terminal_hc_05

/* General definitions   -----------------------------------------------------*/
#define HTS221_SLAVE_ADD    0xBE
#define I2C_MEM_ADD_LSB     (__ADDRESS__)              ((uint8_t)((uint16_t)((__ADDRESS__) & (uint16_t)(0x00FFU))))
#define HTS221_BIT(x) ((uint8_t)x)

/* Private constants ---------------------------------------------------------*/
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


/*******************************************************************************
 * defintions
 *******************************************************************************/
uint8_t Configs (void);

void    i2c_config (void);

void    _bsp_clk_freq_get (void);
void    i2c2_read (uint8_t SADD, uint8_t ReadADD, uint32_t TransferSize, uint8_t *buffer);
void    i2c2_write (uint8_t SADD, uint8_t WriteADD, uint32_t TransferSize, uint8_t *buffer);

uint8_t sensor_read (uint16_t DeviceAddr, uint8_t RegisterAddr);
void    sensor_write (uint16_t DeviceAddr, uint8_t RegisterAddr, uint8_t *tmp);
void    sensor_readmultiple (uint16_t DeviceAddr, uint8_t RegisterAddr, uint8_t *buffer, uint8_t readsize);

void    HTS221_T_Init(uint16_t DeviceAddr);
float   HTS221_T_ReadTemp(uint16_t DeviceAddr);

void    HTS221_H_Init(uint16_t DeviceAddr);
float   HTS221_H_ReadHumidity(uint16_t DeviceAddr);





#endif

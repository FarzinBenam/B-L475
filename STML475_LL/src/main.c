/*
  ******************************************************************************
  * @file
  * @author		Farzin M.Benam
  * @version	v.1 ARM STM32L4
  * @date			2020-06-20
  * @brief
	*						Commander Project
  *******************************************************************************
	* @attention
  *
	* Copyright (C) 2010-2020 Farzin Memaran Benam <Farzin.Memaran@gmail.com>
	*
	* This file is part of Promzit Commander project.
	*
	* This project can not be copied and/or distributed without the express
	* permission of Farzin Memaran Benam.
  *
	* Copyright (C) Promzit Systems, Inc - All Rights Reserved
  * <h2><center>&copy; COPYRIGHT 2020 Farzin_M.Benam</center></h2>
  *******************************************************************************
	* HISTORY:
	*          Date 	| Subject			| Detail
	*    ==========================================================================
	*			2015-12-12	| First Code	| Start recoding the Shell of Commander for General uses
	*			2016-04-01	| Add_cmd     | Process mode changed (Details in Proccess.c)
	*			2020-06-20	| ARM         | ARM Integration Cortex M4
	*			2020-07-21	| WiFi init		| Wifi modude initiated through SPI3
	*			2020-12-04  | SysTick     | SysTick added
	*			2021-01-03	| WiFi        | WiFi interface through uart added
	*			2021-01-26  | Git         | started Git logging
  *     2021.08.01  | LL_Driver   | ST Low-Level Drivers used
	*			2021.08.20	| Quad-spi		| started
	*			2021.09.02	| Log FS			| started
	*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
const char	_TEMPERATURE_HUMIDITY_FOLDER_NAME[] = "T & H";
const char	start_tag[] = "*_";
const char	end_tag[] = "`!";
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Constant definitions ------------------------------------------------------*/


/*
Global status variable
	CmdStatus = 0 -> normal status
	CmdStatus = 1 -> usart int
	CmdStatus = 2 -> command Entered
	CmdStatus = 3 -> cmd without parameter
	CmdStatus = 4 -> cmd with parameter
	CmdStatus = 5 -> wifi ready

	WifiStatus = 0 -> wifi off
	WifiStatus = 1 -> wifi initiated and ready
	WifiStatus = 2 -> wifi recieving respond
	wificmdStatus = 0 -> no wifi command
	wificmdStatus = 1 -> wifi interrupt recieved
	*/
volatile uint8_t CmdStatus = 0;
//volatile uint8_t WifiStatus = 0;
//volatile uint8_t wificmdStatus = 0;
volatile uint32_t tick;



int main(void)
{
  /* Private variables ---------------------------------------------------------*/
  uint8_t		buf[] = "How is the weather inside that chip, Farzin?!";
  uint8_t		temp[2000];
	char			buf1[50];
	char			buf2[3];
  uint32_t	address;
	uint32_t	temp1;
	volatile	uint8_t i;
	
	
	
  fs_folder_def fsfolder;
	LL_RTC_TimeTypeDef time;
	LL_RTC_DateTypeDef date;
  /* Configuration functions ---------------------------------------------------*/
  Configs();
	
	
//	terminal("\nchip erase started");
//	time_show();
//	qspi_Erase_Chip();
//	terminal("\nchip erase ended");
//	time_show();
//	qspi_EraseSector(0x8000, 0x10000);

  T_H_log_init(&fsfolder);





//	qspi_ReadMemory(temp, QSPI_START_ADDRESS, MX25R6435F_SECTOR_SIZE);
//	terminal("\nAddress 0x%X Read:\n", QSPI_START_ADDRESS);
//		for(int k = 0; k < MX25R6435F_SECTOR_SIZE; ){
//			terminal("%.2X ", temp[k++]);
//			if((k % 24) == 0){
//				terminal("_________%X:\n", k);
//			}
//		}

  while (1)
  {
  
//		LED1_ON();
//		LL_mDelay(LED_BLINK_SLOW);
//		LED1_OFF();
//		LL_mDelay(LED_BLINK_SLOW);

//		LED2_ON();
//		LL_mDelay(LED_BLINK_SLOW);
//		LED2_OFF();
//		LL_mDelay(LED_BLINK_SLOW); 

		T_H_log(&fsfolder);
		
		
//		LL_mDelay(2000);
  }
}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

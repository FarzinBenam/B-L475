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
const char	_Error[] = "ERROR: ";
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
volatile uint8_t WifiStatus = 0;
//volatile uint8_t wificmdStatus = 0;
volatile uint32_t tick;



int main(void)
{
  /* Private variables ---------------------------------------------------------*/
  fs_folder_def fsfolder;
	
  /* Configuration functions ---------------------------------------------------*/
  Configs();
	
	

	//qspi_EraseSector(0x8000, 0x10000);
	//qspi_Erase_Chip();

  T_H_log_init(&fsfolder);



	while(1){
		if (CmdStatus == 1) USART_Process();
    if (CmdStatus == 2)	cmd_entry();
    //if (wificmdStatus)  WiFi();
		
		T_H_log(&fsfolder);
		LL_mDelay(2000);
	}


//  while (1)
//  {
//  
//		LED1_ON();
//		LL_mDelay(LED_BLINK_SLOW);
//		LED1_OFF();
//		LL_mDelay(LED_BLINK_SLOW);

//		LED2_ON();
//		LL_mDelay(LED_BLINK_SLOW);
//		LED2_OFF();
//		LL_mDelay(LED_BLINK_SLOW); 

//		T_H_log(&fsfolder);
//		
//		
//		LL_mDelay(2000);
//  }
}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

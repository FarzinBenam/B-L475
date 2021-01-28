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
	*			2016-04-01	| Add_cmd			| Process mode changed (Details in Proccess.c)
	*			2020-06-20	|	ARM					| ARM Integration Cortex M4
	*			2020-07-21	| WiFi init		| Wifi modude initiated through SPI3
	*			2020-12-04  | SysTick			| SysTick added
	*			2021-01-03	| WiFi 				| WiFi interface through uart added
	*			2021-01-26  | Git					| started Git logging
	*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx.h"                  // Device header
#include "../my_libs/fcmd.h"
#include "../my_libs/Processes.h"
//#include "../my_libs/wifi.h"
//#include "../my_libs/es_wifi.h"
//************************************************************************
//* Specific Variable Definitions
//************************************************************************
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
volatile int tick;
volatile int _tick;

volatile uint8_t CmdStatus = 0;
volatile uint8_t WifiStatus = 0;
volatile uint8_t wificmdStatus = 0;



int main (void)
{
  
	OsInits ();
  
  

  
	while(1)
	{

		if (CmdStatus == 1)	USART_Process();
		if (CmdStatus == 2)	cmd_entry();
		if (wificmdStatus) WiFi();

	}
}


 /******************* (C) COPYRIGHT 2020 Farzin_M.Benam *****END OF FILE****/

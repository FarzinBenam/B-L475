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
	*			2016-04-01	| Add_cmd		| Process mode changed (Details in Proccess.c)
	*			2020-06-20	|	ARM			| ARM Integration Cortex M4
	*			2020-07-21	| WiFi init		| Wifi modude initiated through SPI3
	*			2020-12-04  | SysTick		| SysTick added
	*			2021-01-03	| WiFi 			| WiFi interface through uart added
	*			2021-01-26  | Git			| started Git logging
	*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "fcmd.h"





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

TaskHandle_t *const led1TaskHandle;
TaskHandle_t *const led2TaskHandle;
TaskHandle_t *const send1TaskHandle;
TaskHandle_t *const send2TaskHandle;
TaskHandle_t *const CmderStatusPollHandle;
TaskHandle_t *const TimeTerminalHandle;


// create mutex handle
SemaphoreHandle_t uartMutex;
SemaphoreHandle_t CmdStatusMutex;


int main (void)
{
    OsInits ();

    //WIFI_Connect(AT_SET_USER_SSID, AT_SET_USER_PASSPHRASE, 0x03);
    
    
    
    
    // create mutex
    uartMutex = xSemaphoreCreateMutex();
    CmdStatusMutex = xSemaphoreCreateMutex();


    xTaskCreate(led1Task,   NULL, 128, NULL, tskIDLE_PRIORITY, led1TaskHandle);
    xTaskCreate(led2Task,   NULL, 128, NULL, tskIDLE_PRIORITY, led2TaskHandle);
    //xTaskCreate(send1Task,  NULL, 128, NULL, tskIDLE_PRIORITY, send1TaskHandle);
    //xTaskCreate(send2Task,  NULL, 128, NULL, tskIDLE_PRIORITY, send2TaskHandle);  
    xTaskCreate(CmderStatusPoll, NULL, 500, NULL, tskIDLE_PRIORITY, CmderStatusPollHandle);
    //xTaskCreate(TimeTerminal, NULL, 500, NULL, tskIDLE_PRIORITY, TimeTerminalHandle);

    vTaskStartScheduler();        /* Start scheduler */
    
    
    
	while(1)
	{
        
        // Program should never reach here!!!

	}
}
/* ---------------------------------------------------------------------------*/


/* ---------------------------------------------------------------------------*/



 /******************* (C) COPYRIGHT 2020 Farzin_M.Benam *****END OF FILE****/

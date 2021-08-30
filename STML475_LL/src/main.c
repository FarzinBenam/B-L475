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
	*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
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
  uint8_t buf[] = "How is the weather inside that chip, Farzin?!";
  uint8_t temp[100];
  _fs_init_def fs;
  _fs_folder_def fsfolder;
  /* Configuration functions ---------------------------------------------------*/
  Configs();
  
  
  /*----------------------------------------------------------------------------*/
  // test
  qspi_ReadMemory(temp, QSPI_START_ADDRESS, 10);

  if(temp[0] == 0xFF){
    qspi_WriteMemory(buf, QSPI_START_ADDRESS, sizeof(buf));
    terminal("\naddress:0x%X written!", QSPI_START_ADDRESS);
  }
  /*----------------------------------------------------------------------------*/
  
  
  
  
  
  
  
  //qspi_WriteMemory(buf, 100, sizeof(buf));
  
//  qspi_ReadMemory(temp, 100, 50);
//  terminal("\n%s",temp);
//  
//  terminal("\nT: %.2f C", HTS221_T_ReadTemp(HTS221_SLAVE_ADD));
//  terminal("  H: %.2f %%", HTS221_H_ReadHumidity(HTS221_SLAVE_ADD));
  
  
  
  strcpy(fsfolder.Name, "T & H"); // be carefull with the size of array
  strcpy(fsfolder.Commant, "the folder used for stroing the temperature and Humidity logs");
  fsfolder.FolderSize = 40000;
  _fs_new_folder(fsfolder);
  
  
  
  fs.buffer[0] = (uint8_t) (HTS221_T_ReadTemp(HTS221_SLAVE_ADD));
  fs.buffer[1] = (uint8_t) (HTS221_H_ReadHumidity(HTS221_SLAVE_ADD));
  _fs_log(fs);
  
  while (1)
  {
  
//    LED1_ON();
//    LL_mDelay(LED_BLINK_FAST);
//    LED1_OFF();
//    LL_mDelay(LED_BLINK_FAST);
//    
//    LED2_ON();
//    LL_mDelay(LED_BLINK_FAST);
//    LED2_OFF();
//    LL_mDelay(LED_BLINK_FAST);

    
    
  }
}



#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

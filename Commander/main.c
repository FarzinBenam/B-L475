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
	*			2020-06-20	| ARM			| ARM Integration Cortex M4
	*			2020-07-21	| WiFi init		| Wifi modude initiated through SPI3
	*			2020-12-04  | SysTick		| SysTick added
	*			2021-01-03	| WiFi 			| WiFi interface through uart added
	*			2021-01-26  | Git			| started Git logging
	*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "main.h"





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
//volatile int _tick;

volatile uint8_t CmdStatus = 0;
volatile uint8_t WifiStatus = 0;
volatile uint8_t wificmdStatus = 0;

void SystemClock_Config(void);
void Error_Handler(void);
void    mydelay     (int delay);
int main (void)
{
	SystemClock_Config();
	OsInits ();
	//WIFI_GetModuleFwRevision
	nl(1);
	

    
	while(1)
	{ 
		// Program should never reach here!!!
		//terminal("%.2d     \n\r", tick++);
		togglepin(LED1_PORT, LED1_PIN);
		
		terminal("CR1 :%x   \n\r", I2C2->CR1);
		terminal("CR2 :%x   \n\r", I2C2->CR2);
		terminal("OAR1:%x   \n\r", I2C2->OAR1);
		terminal("OAR2:%x   \n\r", I2C2->OAR2);
		terminal("TIMINGR:%x   \n\r", I2C2->TIMINGR);
		terminal("TIMEOUTR:%x   \n\r", I2C2->TIMEOUTR);
		terminal("ISR:%x   \n\r", I2C2->ISR);
		terminal("ICR:%x   \n\r", I2C2->ICR);
		terminal("PECR:%x   \n\r", I2C2->PECR);
		terminal("RXDR:%x   \n\r", I2C2->RXDR);
		terminal("TXDR:%x   \n\r", I2C2->TXDR);

		mydelay(1000);

		
	}
}
void    mydelay     (int delay){
    volatile int i, j;
    for(i = 0; i < delay;i++){
        for(j = 0; j < 1000;j++){
            __NOP();
        }
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
		
  }
  /* USER CODE END Error_Handler_Debug */
}
 /******************* (C) COPYRIGHT 2020 Farzin_M.Benam *****END OF FILE****/

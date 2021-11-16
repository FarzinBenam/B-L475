#include "main.h"



/************************************************************************
 *	Definitions
 ************************************************************************/
extern volatile int tick;
extern volatile int _tick;
volatile int debug = 0;


extern Status	SysStatus;
extern volatile uint8_t	WifiStatus;
extern volatile uint8_t	wificmdStatus;
extern Commands	Cmds[MAX_COMMANDS]; // array of structures

static uint8_t	GernealBuffer[GENERAL_BUFFER_SIZE];
//static uint8_t	IICBuffer[IIC_BUFFER_SIZE];
//static uint8_t	WifiBuffer[WIFI_BUFFER_SIZE];
//I2C_HandleTypeDef hi2c;

//// create mutex handle
//SemaphoreHandle_t uartMutex;
//SemaphoreHandle_t CmdStatusMutex;


/**
  * @brief  
  * @param  None
  * @retval None
  */
//void	XXXX (uint8_t *cmdbuffer)
//{
//	
//	cmd_exit();
//}
/************************************************************************
 * System Processes
 ************************************************************************/
/**
  * @brief  
  * @param  None
  * @retval None
  */
void	_time_cmd (uint8_t *cmdbuffer)
{
	
  // CmdStatus = 2 -> cmd without parameter
	if (SysStatus.CmdStatus == 2){   
    time_show();
  }
	// CmdStatus = 3 -> cmd without parameter
	if (SysStatus.CmdStatus == 3){
		time_set(cmdbuffer);
	}
	
	cmd_exit();
}
/**
  * @brief  
  * @param  None
  * @retval None
  */
void	time_show (void)
{
	LL_RTC_TimeTypeDef	time;
	LL_RTC_DateTypeDef	date;
	
	rtc_read(&time, &date);

	terminal("\n%.2X:%.2X:%.2X ",time.Hours, time.Minutes, time.Seconds);
	if(RTC->TR & RTC_TR_PM){
		terminal("PM");
	}
	terminal("__ 20%.2X-%.2X-%.2X ", date.Year, date.Month, date.Day);
	switch(date.WeekDay){
		case LL_RTC_WEEKDAY_MONDAY:
			terminal("Mon");
			break;
		case LL_RTC_WEEKDAY_TUESDAY:
			terminal("Tues");
			break;
		case LL_RTC_WEEKDAY_WEDNESDAY:
			terminal("Wednes");
			break;
		case LL_RTC_WEEKDAY_THURSDAY:
			terminal("Thurs");
			break;
		case LL_RTC_WEEKDAY_FRIDAY:
			terminal("Fri");
			break;
		case LL_RTC_WEEKDAY_SATURDAY:
			terminal("Satur");
			break;
		case LL_RTC_WEEKDAY_SUNDAY:
			terminal("Sun");
			break;
		
	}
	terminal("day");
}
/**
  * @brief  just read the rtc and show the time
  * @param  None
  * @retval None
  */
void	time_set (uint8_t *cmdbuffer)
{
  LL_RTC_TimeTypeDef time;
	LL_RTC_DateTypeDef date;
  uint8_t buff_idx = 0, i;
	char temp[20];
	int temp1[10];

	unsigned int x, y , z;
	char *ptr;
	
	// clear the time & date structs
	memset(&time, 0, sizeof(time));
	memset(&date, 0, sizeof(date));
	memset(&temp1,0, sizeof(temp1));
	
  
	/* format of setting time:
	** hour minute second year month day weekday
	** example: time 23 15 00 21 10 26 3
	** always SPACE in between numbers
	*/
  /*  !!!!!!!! this method is also true !!!!!!
	sscanf((const char *)cmdbuffer, "%s %d %d %d %d %d %d %d",temp, 
																														&temp1[0], 
																														&temp1[1], 
																														&temp1[2], 
																														&temp1[3],
																														&temp1[4],
																														&temp1[5],
																														&temp1[6]);	
	i = 0;
	time.Hours    = temp1[i++];
	time.Minutes  = temp1[i++];
	time.Seconds	= temp1[i++];
	date.Year			= temp1[i++];
	date.Month		= temp1[i++];
	date.Day			= temp1[i++];
	date.WeekDay	= temp1[i++];
  */
	
  // both above and under method can be applied
  atoi((const char *)strtok((char *)cmdbuffer, " "));     // to skip ke cmd it self
  time.Hours    = atoi((const char *)strtok(NULL, " "));  // NULL as an input mean to countine from where it left
	time.Minutes  = atoi((const char *)strtok(NULL, " "));
	time.Seconds	= atoi((const char *)strtok(NULL, " "));
	date.Year			= atoi((const char *)strtok(NULL, " "));
	date.Month		= atoi((const char *)strtok(NULL, " "));
	date.Day			= atoi((const char *)strtok(NULL, " "));
	date.WeekDay	= atoi((const char *)strtok(NULL, " "));
  
  
//	// use to DEBUG
//	terminal("\n%d %d %d %d %d %d %d",time.Hours, 
//																		time.Minutes,
//																		time.Seconds,
//																		date.Year,
//																		date.Month,
//																		date.Day,
//																		date.WeekDay);
	
  rtc_set(&time, &date);
  time_show();
}
/**
  * @brief  read the given log number
  * @param  log number
  * @retval None
  */
void	_logread_cmd (uint8_t *cmdbuffer)
{
	fs_folder_def fsfolder;
	uint32_t log_num;

	fsfolder.StartAddr = 0;
	fsfolder.EndAddr = 	MX25R6435F_FLASH_SIZE;
	
  // CmdStatus = 3 -> cmd without parameter
	if (SysStatus.CmdStatus == 3){
    
    log_num = param_decode(cmdbuffer);    // extracting the nummber enterred from terminal
    
		if(T_H_log_finder(&fsfolder, log_num) != SUCCESS){
			terminal("\nT_H_log_finder: ERROR!");
		}
  }
  else {
    terminal("\nNo log nummber entered!"); 
  }

	cmd_exit();
}
/**
  * @brief  Full chip erase of the SPI Flash
  * @param  None
  * @retval None
  */
void	_chip_erase_cmd	(uint8_t *cmdbuffer)
{
  qspi_Erase_Chip();
  cmd_exit();
}
/**
  * @brief  Sector erase of the SPI flash
  * @param  Sector address
  * @retval None
  */
void	_sector_erase_cmd	(uint8_t *cmdbuffer)
{
  uint32_t sector_add = param_decode(cmdbuffer);
  
  
  qspi_EraseSector( sector_add/ MX25R6435F_SECTOR_SIZE);
  cmd_exit();
}
/**
  * @brief  temporature
  * @param  None
  * @retval None
  */
void	_hmdty_cmd (uint8_t *cmdbuffer)
{
	float i = HTS221_H_ReadHumidity(HTS221_SLAVE_ADD);
  terminal("\nHumdity:%.2f %%", i);
	cmd_exit();
}
/**
  * @brief  humidity 
  * @param  None
  * @retval None
  */
void	_tmprt_cmd (uint8_t *cmdbuffer)
{
	float i = HTS221_T_ReadTemp(HTS221_SLAVE_ADD);
  terminal("\nTemperature:%.2f `C", i);
  
	cmd_exit();
}

void    Help            (uint8_t *cmdbuffer)
{

  nl(1);
  terminal("List of commands: ");
  nl(1);
  
  for(int i = 0; i < 10; i++){
    nl(1);
    terminal("%s", Cmds[i].cmd_name);
  }
  
  cmd_exit();
}
/**
  * @brief  
  * @param  None
  * @retval None
  */
void	_ (uint8_t *cmdbuffer)
{
	
	cmd_exit();
}













/**
  * @brief  recives commands from terminal, decode, send, and recive the responses
	* 				from the wifi module, shows them on terminal
  * @param  None
  * @retval None
  */
//void	WiFi			(uint8_t *cmdbuffer)
//{
//	
//	volatile int TimeoutCount = 0;
//	volatile int i = 0;
//	char temp = 0;
//	
//	//_usart1_printf("\n\rWiFi: CmdStatus:%d - WifiStatus:%d - wificmdStatus:%d\n\r", CmdStatus, WifiStatus, wificmdStatus);
//	
//	// first if is after recieving the wifi command with parameter
//	if (WifiStatus == 1 && SysStatus.CmdStatus == 2){
//		
//		for(i = 0; cmdbuffer[i] != SPACE; i++);		// finding the start of command (passing "WIFI+SPACE")
//		++i;
//		if(cmdbuffer[i] == 0){				// checking if the command have null parameter! then exit
//			nl(1);
//			terminal("%s", CmdError_2);
//			cmd_exit();
//		}
//		for(;cmdbuffer[i] != 0; i++){	// sending the command in buffer to ISM (wifi module)
//			_usart3_send_b (cmdbuffer[i]);
//			//terminal_send_b (CmdBuffer[i]);
//		}
//		_usart3_send_b (ENTER);				// MUST send an ENTER to apply the command in ISM module
//		
//		WifiStatus = 2;								// means now we expecting respons from ISM module
//		CmdStatus = 0;								// because we coming from CmdEntry() so now exiting
//		CmdLenght = 0;								// because we coming from CmdEntry() so now exiting
//	}
//	else if (WifiStatus == 2){			// recieving the respond from ISM module
//		wificmdStatus = 0;
//		temp = USART3->RDR;
//		
//		
//		if(temp == '>') {							// at the end of respons there is always "> + SPACE"
//			while(wificmdStatus == 0){	// waiting for the end of respons
//				delayMs(1);
//				TimeoutCount++;
//				if(TimeoutCount > 100) goto error;				
//			}
//			if(USART3->RDR == SPACE){
//				cmd_exit();								// restoring every flag to its normal status
//			}
//			else {
//				goto error;
//			}
//		}
//		else {
//			terminal_send_b(temp);				// here we can see the message for debug on terminal
//		}
//	}
//	if(USART3->ISR & IO_5){					// if suddenly an interrupt occured, check this dummy interrupt
//		terminal("unknown state, usart rescieved: 0x%.2x",USART3->RDR);
//		goto error;
//	}
//	
//	
//	wificmdStatus = 0;
//	return;
//	
//	
//error :
//	wificmdStatus = 0;
//	nl(1);
//	terminal("WiFi read Error!");// %d", TimeoutCount);
//	cmd_exit();
//  return;
//}

//void	Wifilisten      (uint8_t *cmdbuffer)
//{
////	char temp;
////    
////    nl(1);
////    terminal("Entering WiFi direct command mode");
////    nl(1);
////    
////    __disable_irq();	// Disable interrupts
////    //taskENTER_CRITICAL();   // Entering Critical Section to just send data exactly to wifi module
////    // HERE WE ARE NOT ALLOWED TO USE RTOS API'S !!!
////    // ALL INTERRUPTS ARE DISABLED HERE !!!

////    while(1){
////        if(USART3->ISR & USART_ISR_RXNE){
////            terminal_send_b(USART3->RDR);
////            USART3->RQR |= (1<<3);
////            GPIOA->ODR ^= LED1_PIN;
////        }

////        if(terminal_usart->ISR & USART_ISR_RXNE){
////            temp = terminal_usart->RDR;
////            if(temp == SCAPE_BUT){
////                nl(1);
////                terminal("Exiting Command Mode");
////                nl(1);
////                break;
////            }
////            terminal_send_b(temp);
////            _usart3_send_b(temp);
////            terminal_usart->RQR |= (1<<3);
////            LED2_PORT->ODR ^= LED2_PIN;
////        }
////    }
////    
////    __enable_irq();
////    //taskEXIT_CRITICAL();    // Exiting Critical Section 
////    
////	cmd_exit();
//}

////************************************************************************
////* RTOS Tasks
////************************************************************************
//void    rtos_Config     (void)
//{
////    TaskHandle_t *const led1TaskHandle;
////    TaskHandle_t *const led2TaskHandle;
////    TaskHandle_t *const send1TaskHandle;
////    TaskHandle_t *const send2TaskHandle;
////    TaskHandle_t *const CmderStatusPollHandle;
////    TaskHandle_t *const TimeTerminalHandle;
////    
////     // create mutex
////    uartMutex = xSemaphoreCreateMutex();
////    CmdStatusMutex = xSemaphoreCreateMutex();
////    
////    //xTaskCreate(led1Task,   NULL, 128, NULL, tskIDLE_PRIORITY, led1TaskHandle);
////    //xTaskCreate(led2Task,   NULL, 128, NULL, tskIDLE_PRIORITY, led2TaskHandle);
////    //xTaskCreate(send1Task,  NULL, 128, NULL, tskIDLE_PRIORITY, send1TaskHandle);
////    //xTaskCreate(send2Task,  NULL, 128, NULL, tskIDLE_PRIORITY, send2TaskHandle);  
////    xTaskCreate(CmderStatusPoll, NULL, 500, NULL, tskIDLE_PRIORITY, CmderStatusPollHandle);
////    //xTaskCreate(TimeTerminal, NULL, 500, NULL, tskIDLE_PRIORITY, TimeTerminalHandle);
////#if (rtos == 1)
////    vTaskStartScheduler();        /* Start scheduler */
////#endif
//}
//void    led1Task        (void *argument)
//{
////  while(1){
////    LED1_PORT->ODR ^= LED1_PIN;
////    vTaskDelay(300 / portTICK_PERIOD_MS);
////    
////    //vTaskDelay(pdMS_TO_TICKS(300));
////  }
//}
//void    led2Task        (void *argument)
//{
////  while(1){
////    LED2_PORT->ODR ^= LED2_PIN;
////      vTaskDelay(305 / portTICK_PERIOD_MS);
////  }
//}
//void    send1Task       (void *argument)
//{
////    while(1){
////        xSemaphoreTake(uartMutex, portMAX_DELAY);
////        terminal("TASK_1: I AM FARZIN MEMARAN BENAM\n\r");
////        xSemaphoreGive(uartMutex);
////        taskYIELD();
////    //vTaskDelay(50 / portTICK_PERIOD_MS);
////    }
//}
//void    send2Task       (void *argument)
//{
////    while(1){
////        if( xSemaphoreTake(uartMutex, 10) == pdTRUE){
////            terminal("task-2; she is somaye, my wife\n\r");
////            xSemaphoreGive(uartMutex);
////            taskYIELD();
////        }
////    //vTaskDelay(20 / portTICK_PERIOD_MS);
////  }
//}
//void    CmderStatusPoll (void *argument)
//{
//    while(1){
//        if (CmdStatus == 1) USART_Process();
//        if (CmdStatus == 2)	cmd_entry();
//        //if (wificmdStatus)  WiFi();
//    }

//}
//void    TimeTerminal    (void *argument)
//{
////    uint8_t dummy[1];
////    while(1){
////        if(CmdStatus == 0){
////            nl(1);
////            BckSpc(2);
////        
////            time(dummy);
////            vTaskDelay(pdMS_TO_TICKS(1000));
////        }
////    }
//}
////************************************************************************
////* System Configurations
////************************************************************************  
//void	RCC_Config		(void)
//{
////    
////    
////	RCC->APB1ENR1 |= (1 << 10); 			// Bit 10 RTCAPBEN: RTC APB clock enable
////	RCC->APB1ENR1 |= (1 << 28);				// Bit 28 PWREN: Power interface clock enable
////	//RCC->APB1ENR1 |= (1 << 15);			// Bit 15 SPI3EN: SPI3 clock enable
////	RCC->APB1ENR1 |= (1 << 18);				// Bit 18 USART3EN: USART3 clock enable
////    RCC->APB1ENR1 |= (1 << 22);             // Bit 22 I2CEN: i2c clock enable
////    __HAL_RCC_UART4_CLK_ENABLE();           // Bit 19 UART4EN: uart4 clock enable
////    //RCC->APB1ENR1 |= (1 << 19);           // Bit 19 UART4EN: uart4 clock enable
////	
////	// this bit is enabled in ISR_Config() for better understanding
//////	RCC->APB2ENR	|= RCC_APB2ENR_SYSCFGEN;// Bit 0 SYSCFGEN: SYSCFG + COMP + VREFBUF clock enable
////	RCC->APB2ENR	|= RCC_APB2ENR_USART1EN;// Bit 14 USART1EN: USART1clock enable
////	
////    /* GPIO Ports Clock Enable */
////    __HAL_RCC_GPIOA_CLK_ENABLE();
////    __HAL_RCC_GPIOB_CLK_ENABLE();
////    __HAL_RCC_GPIOC_CLK_ENABLE();
////    __HAL_RCC_GPIOD_CLK_ENABLE();
////    __HAL_RCC_GPIOE_CLK_ENABLE();
////    /* the same code above with out HAL
////	RCC->AHB2ENR	|= RCC_AHB2ENR_GPIOAEN;	// Bit 0 GPIOAEN: IO port A clock enable
////	RCC->AHB2ENR	|= RCC_AHB2ENR_GPIOBEN;	// Bit 1 GPIOBEN: IO port B clock enable
////	RCC->AHB2ENR	|= RCC_AHB2ENR_GPIOCEN;	// Bit 2 GPIOCEN: IO port C clock enable
////	RCC->AHB2ENR	|= RCC_AHB2ENR_GPIODEN;	// Bit 3 GPIODEN: IO port D clock enable
////	RCC->AHB2ENR	|= RCC_AHB2ENR_GPIOEEN;	// Bit 4 GPIOEEN: IO port E clock enable
////	*/
////	
////    
////    
////	
//	
//	
//}



//void	ISR_Config		(void)
//{
////	RCC->APB2ENR	|= RCC_APB2ENR_SYSCFGEN;// Bit 0 SYSCFGEN: SYSCFG + COMP + VREFBUF clock enable

////	//External Interrupts (EXTI)
////    /*******************************************************************************/  
////	// EXTI for PE1 (WiFi RDY Pin)
////	SYSCFG->EXTICR[0] &= ~(IO_4|IO_5|IO_6|IO_7); 
////	SYSCFG->EXTICR[0] = (1<<6); 
////	EXTI->RTSR1				|= WIFI_RDY_PIN;
////	EXTI->FTSR1				&= ~WIFI_RDY_PIN;
////	EXTI->IMR1				|= WIFI_RDY_PIN;
////	NVIC_SetPriority	(EXTI1_IRQn, 1);			// Priority level 1
////	NVIC_EnableIRQ		(EXTI1_IRQn);
////	
////	
////	//EXTI13 external interrupt
////	// select the source input for the EXTI13 external interrupt.
////	// EXTICR[3] -> because we count from 0 then EXTICR[3] = EXTICR[4] here
////	SYSCFG->EXTICR[3] &= ~(IO_4|IO_5|IO_6|IO_7); 
////	SYSCFG->EXTICR[3] = (1<<5); 
////	EXTI->RTSR1				&= ~PB1_PIN;
////	EXTI->FTSR1				|= PB1_PIN;
////	EXTI->IMR1				|= PB1_PIN;
////	NVIC_SetPriority	(EXTI15_10_IRQn, 1);	    // Priority level 1
////	NVIC_EnableIRQ		(EXTI15_10_IRQn);
////	
////	
////	// USART1 (stlink) or UART4 (HC-05) interrupt
////	/*******************************************************************************/  
////	// A USART interrupt is generated whenever ORE=1 or RXNE=1 in the USART_ISR register
////	terminal_usart->CR1		|= (1 << 5); 			// RXNE interrupt enable
////	NVIC_SetPriority	(terminal_IRQn, 1); 		// Priority level 1
////	NVIC_EnableIRQ		(terminal_IRQn);
////	
////	// USART3 interrupt  (ISM43362-M3G-L44 wifi module)
////	/*******************************************************************************/  
////	// A USART interrupt is generated whenever ORE=1 or RXNE=1 in the USART_ISR register
////	USART3->CR1		|= (1 << 5); 					// RXNE interrupt enable
////	NVIC_SetPriority	(USART3_IRQn, 1); 			// Priority level 1
////	NVIC_EnableIRQ		(USART3_IRQn);
////    
////	
//	/*******************************************************************************/
//}

//void	RTC_Config		(void)
//{
////	PWR->CR1	|= (1<<8);									// Bit 8 DBP: Disable backup domain write protection
////	RCC->BDCR |= (1<<0);									// Bit 0 LSEON: LSE oscillator enable
////	RCC->BDCR |= (1<<8);									// Bits 9:8 RTCSEL[1:0]: RTC clock source selection
////	RCC->BDCR &= ~IO_9;										// Bits 9:8 RTCSEL[1:0]: RTC clock source selection
////	RCC->BDCR |= (1<<15);									// Bit 15 RTCEN: RTC clock enable
////	
//}

//void	Systick_EN		(int tick)
//{
//	SysTick->LOAD = tick-1;
//	SysTick->VAL = 0;
//	SysTick->CTRL = 7;
//}
//uint32_t getTick		(void)
//{
////	__disable_irq();
////	_tick = tick;
////	__enable_irq();
//	
////	return _tick;
//    return 0;
//}
//void	wifi_wakeup		(void)
//{
////	volatile int TimeoutCount = 0;
////	volatile int ack = 0;
////	char temp = 0;
////	
////	
////	// Initiating the wifi module at startup
////	/* goal of this while loop is to recieve welcome message of the wifi module
////	// after reset.
////	// this loop exits successfully when the message comes with th '>'+SPACE at the
////	// end of it. */
////	WIFI_RST();
////	LED3_ON();		// wifi LED on
////	WifiStatus = 1;
////	

////	while(1){
////		wificmdStatus = 0;

////		temp = USART3->RDR;
////		//terminal_send_b(temp);		// here we can see the message for debug
////		
////		if(temp == '>') ack = 1;
////		if(ack){
////			if(temp == SPACE) goto Done;
////		}
////		
////		++TimeoutCount;
////		if(TimeoutCount > 150) goto error;
////		while(wificmdStatus == 0);
////	}
////	
////error :
////	wificmdStatus = 0;
////	nl(1);
////	terminal("WiFi Init Error!");// %d", TimeoutCount);
////  return;	
////	
////	
////Done :
////    terminal("WiFi OK!");
////	wificmdStatus = 0;
////	return;
//}
//void    I2C2_Config     (void)
//{
////	I2C_HandleTypeDef hi2c;
////	
////	// Configure the I2C peripheral
////	__HAL_RCC_I2C2_CLK_ENABLE();        //RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;
////	__HAL_RCC_I2C2_FORCE_RESET();       //SET_BIT(RCC->APB1RSTR1, RCC_APB1RSTR1_I2C2RST)
////	__HAL_RCC_I2C2_RELEASE_RESET();     //CLEAR_BIT(RCC->APB1RSTR1, RCC_APB1RSTR1_I2C2RST)

//////	I2C2->CR1 = 0;          				// disale everything
//////	//relaxed timing, adjust for your bus frequency
//////	I2C2->TIMINGR = ((uint32_t)0x00702681);
//////	I2C2->OAR1 = 0;         				// OwnAddress1
//////	I2C2->OAR2 = 0;         				// OwnAddress2
//////	I2C2->CR2 |= (I2C_CR2_AUTOEND);	// Automatic end mode (master mode)
//////	I2C2->CR1 |= (I2C_CR1_PE	);    // I2C2 Peripheral enable


////	/* Enable and set I2Cx Interrupt to a lower priority */
////	HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0x0F, 0);
////	HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);

////	/* Enable and set I2Cx Interrupt to a lower priority */
////	HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0x0F, 0);
////	HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);

////	
////	/* I2C configuration */
////  hi2c.Instance              = I2C2;
////  hi2c.Init.Timing           = ((uint32_t)0x00702681); //0111 0000 0010 0110 1000 0001
////  hi2c.Init.OwnAddress1      = 0;
////  hi2c.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
////  hi2c.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
////  hi2c.Init.OwnAddress2      = 0;
////  hi2c.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
////  hi2c.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

////  /* Init the I2C */
////  //I2Cx_MspInit(&hi2c);
////  HAL_I2C_Init(&hi2c);
////  
////  /**Configure Analogue filter */
////  HAL_I2CEx_ConfigAnalogFilter(&hi2c, I2C_ANALOGFILTER_ENABLE); 
//}    
////************************************************************************
////* Pheripheral Functions
////************************************************************************

///**
//  * @brief  
//  * @param  None
//  * @retval None
//  */
//void	wifi_Connect	(void)
//{
////    char *SSID = "Vodafone-F919";
////	char *PASSWD = "EwAMuAMNPrHHTuNh";
////    char req_data[BUF_SZ];
////    
////	printf("WIFI Init ...\n\r");
////    WIFI_Status_t ret = WIFI_Init();

////    if (ret != WIFI_STATUS_OK) {
////    printf("Failed to initialize Wi-Fi [%d]\n\r", ret);
////    }

////    uint8_t  MAC_Addr[6];
////    if(WIFI_GetMAC_Address(MAC_Addr) == WIFI_STATUS_OK)
////    {
////    printf("WIFI MAC Address : %X:%X:%X:%X:%X:%X\n\r",
////    MAC_Addr[0], MAC_Addr[1], MAC_Addr[2],
////    MAC_Addr[3], MAC_Addr[4], MAC_Addr[5]);
////    }
////    else
////    {
////    printf("> ERROR : CANNOT get MAC address\n\r");
////    }

////    printf("WIFI Connect to %s ...\n\r", SSID);
////    ret = WIFI_Connect(SSID, PASSWD, WIFI_ECN_WPA2_PSK);
////    if (ret != WIFI_STATUS_OK)
////    {
////    printf("Failed to connect AP [%d]\n\r", ret);
////    }

////    uint8_t IP_Addr[4];
////    if(WIFI_GetIP_Address(IP_Addr) == WIFI_STATUS_OK)
////    {
////    printf("WIFI IP Address : %.3d.%.3d.%.3d.%.3d", IP_Addr[0],IP_Addr[1],IP_Addr[2],IP_Addr[3]);

////    }


//}

///**
//  * @brief  
//  * @param  None
//  * @retval None
//  */
//void	time_UTC		(int epoch)
//{
////	unsigned char ntp_hour, ntp_minute, ntp_second, ntp_week_day, ntp_date, ntp_month, leap_days, leap_year_ind ;
////	unsigned short temp_days;
////	unsigned int ntp_year, days_since_epoch, day_of_year; 
////	char key;
////	
//////---------------------------- Input and Calculations -------------------------------------

////	leap_days=0; 
////	leap_year_ind=0;
////	nl(1);
////	terminal("------------------------------\n");    
//////	_usart1_printf("EPOCH => %d",&epoch);

////	// Add or substract time zone here. 
////	epoch += 7200 ; //GMT +2:00 = +7200 seconds (Berlin Time) 

////	ntp_second = epoch%60;
////	epoch /= 60;
////	ntp_minute = epoch%60;
////	epoch /= 60;
////	ntp_hour  = epoch%24;
////	epoch /= 24;
////		
////	days_since_epoch = epoch;      //number of days since epoch
////	ntp_week_day = week_days[days_since_epoch%7];  //Calculating WeekDay
////	
////	ntp_year = 1970+(days_since_epoch/365); // ball parking year, may not be accurate!

////	int i;
////	for (i=1972; i<ntp_year; i+=4)      // Calculating number of leap days since epoch/1970
////		 if(((i%4==0) && (i%100!=0)) || (i%400==0)) leap_days++;
////				
////	ntp_year = 1970 + ((days_since_epoch - leap_days)/365); // Calculating accurate current year by (days_since_epoch - extra leap days)
////	day_of_year = ((days_since_epoch - leap_days)%365) + 1;


////	if(((ntp_year%4==0) && (ntp_year%100!=0)) || (ntp_year%400==0))  
////	 {
////		 month_days[1]=29;     //February = 29 days for leap years
////		 leap_year_ind = 1;    //if current year is leap, set indicator to 1 
////		}
////				else month_days[1]=28; //February = 28 days for non-leap years 

////				temp_days=0;

////	for (ntp_month=0 ; ntp_month <= 11 ; ntp_month++) //calculating current Month
////		{
////			if (day_of_year <= temp_days) break; 
////			temp_days = temp_days + month_days[ntp_month];
////		}

////	temp_days = temp_days - month_days[ntp_month-1]; //calculating current Date
////	ntp_date = day_of_year - temp_days;

////	// -------------------- Printing Results ------------------------------------- 

////	switch(ntp_week_day) {
////											 
////											 case 0:  terminal("Sunday");
////															 break;
////											 case 1:  terminal("Monday");
////															 break;
////											 case 2:  terminal("Tuesday");
////															 break;
////											 case 3:  terminal("Wednesday");
////															 break;
////											 case 4:  terminal("Thursday");
////															 break;
////											 case 5:  terminal("Friday");
////															 break;
////											 case 6:  terminal("Saturday");
////															 break;
////											 default: break;        
////											 }
////	 terminal(", "); 

////	switch(ntp_month) {
////												 
////												 case 1:  terminal("January");
////																 break;
////												 case 2:  terminal("February");
////																 break;
////												 case 3:  terminal("March");
////																 break;
////												 case 4:  terminal("April");
////																 break;
////												 case 5:  terminal("May");
////																 break;
////												 case 6:  terminal("June");
////																 break;
////												 case 7:  terminal("July");
////																 break;
////												 case 8:  terminal("August");
////																 break;
////												 case 9:  terminal("September");
////																 break;
////												 case 10:  terminal("October");
////																 break;
////												 case 11:  terminal("November");
////																 break;
////												 case 12:  terminal("December");       
////												 default: break;        
////												 }

////	 terminal(" %2d",ntp_date);
////	 terminal(", %d\n",ntp_year);
////	 terminal("TIME = %2d : %2d : %2d\n\r", ntp_hour,ntp_minute,ntp_second)  ;
//////	 _usart1_printf("Days since Epoch: %d\n",days_since_epoch);
//////	 _usart1_printf("Number of Leap days since EPOCH: %d\n",leap_days);
//////	 _usart1_printf("Day of year = %d\n", day_of_year);
//////	 _usart1_printf("Is Year Leap? %d\n",leap_year_ind);
////	 terminal("===============================\n");
////	
////	cmd_exit();
////  return;
//}
////************************************************************************
////* System Functions
////************************************************************************
//void	OsInits			(void)
//{
//	__disable_irq();	// Disable interrupts

//	//Systick_EN(OneMilliSec);
//	RCC_Config();
//	GPIO_Config();
//	terminal_Config();
//	//USART1_Config();
//	I2C2_Config();



//	RTC_Config();
//	//SPI3_Config();
//	ISR_Config();



//	//wifi_wakeup();
//	Welcome();
//	//wifi_Connect();
//	//__enable_irq();
//	//rtos_Config();
//	
//	
//	
//	
//	
//	
//	
//	
//	
//	uint8_t read_value = 0;

//  HAL_I2C_Mem_Read(&hi2c, 0xBF, (uint16_t)0x20, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&read_value, 1, 1000);

//	
//}

//void	_wifi_send      (const char *format, ...)
//{
//	static  uint8_t  buffer[40 + 1];
//	va_list     vArgs;
//		
//	va_start(vArgs, format);
//	vsprintf((char *)buffer, (char const *)format, vArgs);
//	va_end(vArgs);
//	

//	_usart3_send_s((char *) buffer);
//}
//int		_wifi_read      (void)
//{
//	while(!(USART3->ISR & IO_5));					// RXNE: Read data register not empty
//	
//	return USART3->RDR;
//}


//void	delayS			(int Seconds )
//{
////	int i;
////	
////	// Because we counting the milliseconds ix1000
////	Seconds *= 1000;
////	i = getTick();
////	
////	while((getTick()- i) < Seconds); 
//}
//void	delayMs			(int MilliSecond)
//{
////	int i;
////	
////	// Because we counting the milliseconds ix1000
////	i = getTick();
////	
////	while((getTick()- i) < MilliSecond); 
//}

//void	halt			(int num)
//{	
//	//while(GPIOC->IDR & IO_13)
//	nl(1);
//	//terminal("Halt_%d", num);
//	while(num--)
//	{
//		//LED1_ON;													// BSRR register low byte is as set
//		LED2_OFF();													// BSRR register high byte is as reset
//		delayMs(10);

//		//LED1_OFF;
//		LED2_ON();
//		delayMs(10);
//	}
//	LED1_OFF();
//	LED2_OFF();
//}

//void	Welcome		(void)
//{
//	int i;
//	
//	nl(2); 
//	terminal_send_s(wc_note);
//	line(2);
//	nl(1);
//	
//	terminal_send_b(CommandSign);
//}


//void	nl				(int Count)
//{
//	int i;
//	for (i = 0 ; i < Count ; i++) terminal_send_s(New_Line);
//}
//void	line			(int leng)
//{
//	if(leng == 1){
//		nl(1);
//		terminal_send_s("________");
//	} else {
//		nl(1);
//		terminal_send_s("_______________");
//	}
//}

//void	BckSpc			(uint8_t Line)
//{
//	uint8_t i;
//	
//	for(i = 0; i < Line; i++){
//		terminal_send_b(ENTER);
//		terminal_send_b(BACKSPACE1);
//	}
//	terminal_send_b(ENTER);
//}

//void	leds			(void)
//{
//	GPIOC->BSRR |= IO_4;
//	GPIOC->BRR |= IO_5;
//	delayMs(30);
//	
//	GPIOC->BRR |= IO_4;
//	GPIOC->BSRR |= IO_5;
//	delayMs(30);
//	
//	GPIOC->BSRR |= IO_4;
//	GPIOC->BSRR |= IO_5;
//	delayMs(30);
//	
//	
//}
//void	led1			(void)
//{
//	while(1){
//		LED1_ON();
//		delayMs(300);
//		
//		LED1_OFF();
//		delayMs(300);
//	}
//}
//void	led2			(void)
//{
//	while(1){
//		LED2_ON();
//		delayMs(300);
//		
//		LED2_OFF();
//		delayMs(300);
//	}
//}

//void    togglepin       (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
//{
//  GPIOx->ODR ^= GPIO_Pin;
//}


///******************************************************************************/
///* STM32L4xx Peripheral Interrupt Handlers                                    */
///* Add here the Interrupt Handlers for the used peripherals.                  */
///* For the available peripheral interrupt handler names,                      */
///* please refer to the startup file (startup_stm32l4xx.s).                    */
///******************************************************************************/
//void EXTI1_IRQHandler(void)
//{
//    CmdStatus = 5;

//	terminal("\n\rim at EXTI1_IRQHandler");
//	EXTI->PR1 |= EXTI_PR1_PIF1;			// Pending interrupt flag on line x (x = 16 to 0)
//	
//    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
//}

//void EXTI9_5_IRQHandler(void)
//{
//    terminal("\n\rim at EXTI9_5_IRQHandler");
//    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
//    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
//    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
//    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
//}

//void EXTI15_10_IRQHandler(void)
//{
//    nl(1);
//	terminal("Button Pushed!");
//	EXTI->PR1 |= EXTI_PR1_PIF13;		// Pending interrupt flag on line x (x = 16 to 0)
//	
//	nl(1);
//	terminal_send_b(CommandSign);
//    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
//    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
//    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
//    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
//    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
//    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
//}

//void SPI3_IRQHandler(void)
//{
//    terminal("\n\rim at SPI3_IRQHandler");
//    HAL_SPI_IRQHandler(&hspi3);
//}

//void	USART1_IRQHandler		(void)
//{
//    togglepin(GPIOB,14);
//	CmdStatus = 1;
//	USART1->RQR |= (1<<3);
//}
//void	USART3_IRQHandler		(void)
//{
//	wificmdStatus = 1;
//	USART3->RQR |= (1<<3);
//}
//void    UART4_IRQHandler        (void)
//{
//    //terminal("\n\rim at UART4_IRQHandler");
//    //LED1_PORT->ODR ^= LED1_PIN;
//    CmdStatus = 1;
//    UART4->RQR |= (1<<3);
//}
////void SysTick_Handler			(void)
////{
////	++tick;
////	 //_usart1_printf("\n\rtick: %d!", tick);
////	// GPIOA->ODR ^= LD1;
////}

////************************************************************************
































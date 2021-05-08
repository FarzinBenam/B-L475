#include "main.h"



//************************************************************************
//*	Definitions
//************************************************************************
extern volatile int tick;
extern volatile int _tick;
volatile int debug = 0;

extern volatile uint8_t	CmdStatus;
extern volatile uint8_t	WifiStatus;
extern volatile uint8_t	wificmdStatus;


static uint8_t	IICBuffer[IIC_BUFFER_SIZE];
static uint8_t	WifiBuffer[WIFI_BUFFER_SIZE];


extern SemaphoreHandle_t uartMutex;
extern SemaphoreHandle_t CmdStatusMutex;



//************************************************************************
//* System Processes
//************************************************************************
/**
  * @brief  
  * @param  None
  * @retval None
  */
void	time			(uint8_t *cmdbuffer)
{
	RTC_t	time;
	int temp;
    
    xSemaphoreTake(CmdStatusMutex, portMAX_DELAY);
	
	time.sec = RTC->TR;
	time.min = (RTC->TR >> 8);
	time.hour = (RTC->TR >> 16);
	time.hour &= ~(IO_6);
	
	time.mday = RTC->DR;
	time.month = (RTC->DR >> 8);
	time.month &= ~(IO_7|IO_6|IO_5);
	time.year = (RTC->DR >> 16);
	
	
	//nl(1);
	//_usart1_printf("Time: %.2X:%.2X:%.2X ",time.hour, time.min, time.sec);
	terminal("%.2X:%.2X:%.2X ",time.hour, time.min, time.sec);
	if(RTC->TR & IO_22) terminal("PM");

	//nl(1);
	//_usart1_printf("Date: 20%.2X:%.2X:%.2X ", time.year, time.month, time.mday);
	
	cmd_exit();
    
    xSemaphoreGive(CmdStatusMutex);
}
/**
  * @brief  
  * @param  None
  * @retval None
  */
void	Set_time		(uint8_t *cmdbuffer)
{
	//RTC->WPR	= 0xCA;				// RTC write protection register
	//RTC->WPR	= 0x53;				// RTC write protection register
}
/**
  * @brief  
  * @param  None
  * @retval None
  */
void	time_UTC		(int epoch)
{
	unsigned char ntp_hour, ntp_minute, ntp_second, ntp_week_day, ntp_date, ntp_month, leap_days, leap_year_ind ;
	unsigned short temp_days;
	unsigned int ntp_year, days_since_epoch, day_of_year; 
	char key;
	
//---------------------------- Input and Calculations -------------------------------------

	leap_days=0; 
	leap_year_ind=0;
	nl(1);
	terminal("------------------------------\n");    
//	_usart1_printf("EPOCH => %d",&epoch);

	// Add or substract time zone here. 
	epoch += 7200 ; //GMT +2:00 = +7200 seconds (Berlin Time) 

	ntp_second = epoch%60;
	epoch /= 60;
	ntp_minute = epoch%60;
	epoch /= 60;
	ntp_hour  = epoch%24;
	epoch /= 24;
		
	days_since_epoch = epoch;      //number of days since epoch
	ntp_week_day = week_days[days_since_epoch%7];  //Calculating WeekDay
	
	ntp_year = 1970+(days_since_epoch/365); // ball parking year, may not be accurate!

	int i;
	for (i=1972; i<ntp_year; i+=4)      // Calculating number of leap days since epoch/1970
		 if(((i%4==0) && (i%100!=0)) || (i%400==0)) leap_days++;
				
	ntp_year = 1970 + ((days_since_epoch - leap_days)/365); // Calculating accurate current year by (days_since_epoch - extra leap days)
	day_of_year = ((days_since_epoch - leap_days)%365) + 1;


	if(((ntp_year%4==0) && (ntp_year%100!=0)) || (ntp_year%400==0))  
	 {
		 month_days[1]=29;     //February = 29 days for leap years
		 leap_year_ind = 1;    //if current year is leap, set indicator to 1 
		}
				else month_days[1]=28; //February = 28 days for non-leap years 

				temp_days=0;

	for (ntp_month=0 ; ntp_month <= 11 ; ntp_month++) //calculating current Month
		{
			if (day_of_year <= temp_days) break; 
			temp_days = temp_days + month_days[ntp_month];
		}

	temp_days = temp_days - month_days[ntp_month-1]; //calculating current Date
	ntp_date = day_of_year - temp_days;

	// -------------------- Printing Results ------------------------------------- 

	switch(ntp_week_day) {
											 
											 case 0:  terminal("Sunday");
															 break;
											 case 1:  terminal("Monday");
															 break;
											 case 2:  terminal("Tuesday");
															 break;
											 case 3:  terminal("Wednesday");
															 break;
											 case 4:  terminal("Thursday");
															 break;
											 case 5:  terminal("Friday");
															 break;
											 case 6:  terminal("Saturday");
															 break;
											 default: break;        
											 }
	 terminal(", "); 

	switch(ntp_month) {
												 
												 case 1:  terminal("January");
																 break;
												 case 2:  terminal("February");
																 break;
												 case 3:  terminal("March");
																 break;
												 case 4:  terminal("April");
																 break;
												 case 5:  terminal("May");
																 break;
												 case 6:  terminal("June");
																 break;
												 case 7:  terminal("July");
																 break;
												 case 8:  terminal("August");
																 break;
												 case 9:  terminal("September");
																 break;
												 case 10:  terminal("October");
																 break;
												 case 11:  terminal("November");
																 break;
												 case 12:  terminal("December");       
												 default: break;        
												 }

	 terminal(" %2d",ntp_date);
	 terminal(", %d\n",ntp_year);
	 terminal("TIME = %2d : %2d : %2d\n\r", ntp_hour,ntp_minute,ntp_second)  ;
//	 _usart1_printf("Days since Epoch: %d\n",days_since_epoch);
//	 _usart1_printf("Number of Leap days since EPOCH: %d\n",leap_days);
//	 _usart1_printf("Day of year = %d\n", day_of_year);
//	 _usart1_printf("Is Year Leap? %d\n",leap_year_ind);
	 terminal("===============================\n");
	
	cmd_exit();
  return;
}
/**
  * @brief  recives commands from terminal, decode, send, and recive the responses
	* 				from the wifi module, shows them on terminal
  * @param  None
  * @retval None
  */
void	WiFi			(uint8_t *cmdbuffer)
{
//	
//    volatile int TimeoutCount = 0;
//	volatile int i = 0;
//	char temp = 0;
//	
//	//_usart1_printf("\n\rWiFi: CmdStatus:%d - WifiStatus:%d - wificmdStatus:%d\n\r", CmdStatus, WifiStatus, wificmdStatus);
//	
//	// first if is after recieving the wifi command with parameter
//	if (WifiStatus == 1 && CmdStatus == 2){
//		
//		for(i = 0; cmdbuffer[i] != SPACE; i++);		// finding the start of command (passing "WIFI+SPACE")
//		++i;
//		if(cmdbuffer[i] == 0){				// checking if the command have null parameter! then exit
//			nl(1);
//			write(CmdError_2);
//			cmd_exit();
//		}
//		for(;cmdbuffer[i] != 0; i++){	// sending the command in buffer to ISM (wifi module)
//			_usart3_send_b (cmdbuffer[i]);
//			//_usart1_send_b (CmdBuffer[i]);
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
//			_usart1_send_b(temp);				// here we can see the message for debug on terminal
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
}

void	Wifilisten      (uint8_t *cmdbuffer)
{
	char temp;
    
    nl(1);
    terminal("Entering WiFi direct command mode");
    nl(1);
    
    __disable_irq();	// Disable interrupts
    //taskENTER_CRITICAL();   // Entering Critical Section to just send data exactly to wifi module
    // HERE WE ARE NOT ALLOWED TO USE RTOS API'S !!!
    // ALL INTERRUPTS ARE DISABLED HERE !!!

    while(1){
        if(USART3->ISR & USART_ISR_RXNE){
            _usart1_send_b(USART3->RDR);
            USART3->RQR |= (1<<3);
            GPIOA->ODR ^= LD1_PIN;
        }

        if(USART1->ISR & USART_ISR_RXNE){
            temp = USART1->RDR;
            if(temp == SCAPE_BUT){
                nl(1);
                terminal("Exiting Command Mode");
                nl(1);
                break;
            }
            _usart1_send_b(temp);
            _usart3_send_b(temp);
            USART1->RQR |= (1<<3);
            GPIOB->ODR ^= LD2_PIN;
        }
    }
    
    __enable_irq();
    //taskEXIT_CRITICAL();    // Exiting Critical Section 
    
	cmd_exit();
}
void    Help            (uint8_t *cmdbuffer)
{
//  int i = 0;
//  nl(1);
//  terminal("List of commands: ");
//  nl(1);
//  
//  for(; i < 10; i++){
//    nl(1);
//    terminal("%s", Cmds[i].Command);
//  }
//  
//  cmd_exit();
}
//************************************************************************
//* RTOS Tasks
//************************************************************************
void    led1Task        (void *argument)
{
  while(1){
    LD1_PORT->ODR ^= LD1_PIN;
    vTaskDelay(300 / portTICK_PERIOD_MS);
    
    //vTaskDelay(pdMS_TO_TICKS(300));
  }
}
void    led2Task        (void *argument)
{
  while(1){
    LD2_PORT->ODR ^= LD2_PIN;
      vTaskDelay(305 / portTICK_PERIOD_MS);
  }
}
void    send1Task       (void *argument)
{
    while(1){
        xSemaphoreTake(uartMutex, portMAX_DELAY);
        terminal("TASK_1: I AM FARZIN MEMARAN BENAM\n\r");
        xSemaphoreGive(uartMutex);
        taskYIELD();
    //vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}
void    send2Task       (void *argument)
{
    while(1){
        if( xSemaphoreTake(uartMutex, 10) == pdTRUE){
            terminal("task-2; she is somaye, my wife\n\r");
            xSemaphoreGive(uartMutex);
            taskYIELD();
        }
    //vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}
void    CmderStatusPoll (void *argument)
{
    while(1){
        if (CmdStatus == 1) USART_Process();
        if (CmdStatus == 2)	cmd_entry();
        //if (wificmdStatus)  WiFi();
    }

}
void    TimeTerminal    (void *argument)
{
    uint8_t dummy[1];
    while(1){
        if(CmdStatus == 0){
            nl(1);
            BckSpc(2);
        
            time(dummy);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}
//************************************************************************
//* System Configurations
//************************************************************************  
void	RCC_Config		(void)
{
	RCC->APB1ENR1 |= (1 << 10); 					// Bit 10 RTCAPBEN: RTC APB clock enable
	RCC->APB1ENR1 |= (1 << 28);						// Bit 28 PWREN: Power interface clock enable
	//RCC->APB1ENR1 |= (1 << 15);					    // Bit 15 SPI3EN: SPI3 clock enable
	RCC->APB1ENR1 |= (1 << 18);						// Bit 18 USART3EN: USART3 clock enable
    RCC->APB1ENR1 |= (1 << 22);                     // Bit 22 I2CEN: i2c clock enable
	
	// this bit is enabled in ISR_Config() for better understanding
//	RCC->APB2ENR	|= RCC_APB2ENR_SYSCFGEN;// Bit 0 SYSCFGEN: SYSCFG + COMP + VREFBUF clock enable
	RCC->APB2ENR	|= RCC_APB2ENR_USART1EN;// Bit 14 USART1EN: USART1clock enable
	
	RCC->AHB2ENR	|= RCC_AHB2ENR_GPIOAEN;	// Bit 0 GPIOAEN: IO port A clock enable
	RCC->AHB2ENR	|= RCC_AHB2ENR_GPIOBEN;	// Bit 1 GPIOBEN: IO port B clock enable
	RCC->AHB2ENR	|= RCC_AHB2ENR_GPIOCEN;	// Bit 2 GPIOCEN: IO port C clock enable
	RCC->AHB2ENR	|= RCC_AHB2ENR_GPIODEN;	// Bit 3 GPIODEN: IO port D clock enable
	RCC->AHB2ENR	|= RCC_AHB2ENR_GPIOEEN;	// Bit 4 GPIOEEN: IO port E clock enable
	
	
	
	
	
	
}



void	ISR_Config		(void)
{
	RCC->APB2ENR	|= RCC_APB2ENR_SYSCFGEN;// Bit 0 SYSCFGEN: SYSCFG + COMP + VREFBUF clock enable

	//External Interrupts (EXTI)
	/*******************************************************************************/  
	// EXTI for PE1 (WiFi RDY Pin)
	SYSCFG->EXTICR[0] &= ~(IO_4|IO_5|IO_6|IO_7); 
	SYSCFG->EXTICR[0] = (1<<6); 
	EXTI->RTSR1				|= WIFI_RDY_PIN;
	EXTI->FTSR1				&= ~WIFI_RDY_PIN;
	EXTI->IMR1				|= WIFI_RDY_PIN;
	NVIC_SetPriority	(EXTI1_IRQn, 1);			// Priority level 1
	NVIC_EnableIRQ		(EXTI1_IRQn);
	
	
	//EXTI13 external interrupt
	// select the source input for the EXTI13 external interrupt.
	// EXTICR[3] -> because we count from 0 then EXTICR[3] = EXTICR[4] here
	SYSCFG->EXTICR[3] &= ~(IO_4|IO_5|IO_6|IO_7); 
	SYSCFG->EXTICR[3] = (1<<5); 
	EXTI->RTSR1				&= ~PB1;
	EXTI->FTSR1				|= PB1;
	EXTI->IMR1				|= PB1;
	NVIC_SetPriority	(EXTI15_10_IRQn, 1);	// Priority level 1
	NVIC_EnableIRQ		(EXTI15_10_IRQn);
	
	
	// USART1 interrupt
	/*******************************************************************************/  
	// A USART interrupt is generated whenever ORE=1 or RXNE=1 in the USART_ISR register
	USART1->CR1		|= (1 << 5); 								// RXNE interrupt enable
	NVIC_SetPriority	(USART1_IRQn, 1); 			// Priority level 1
	NVIC_EnableIRQ		(USART1_IRQn);
	
	// USART3 interrupt  (ISM43362-M3G-L44 wifi module)
	/*******************************************************************************/  
	// A USART interrupt is generated whenever ORE=1 or RXNE=1 in the USART_ISR register
	USART3->CR1		|= (1 << 5); 								// RXNE interrupt enable
	NVIC_SetPriority	(USART3_IRQn, 1); 			// Priority level 1
	NVIC_EnableIRQ		(USART3_IRQn);
	
	/*******************************************************************************/
}

void	RTC_Config		(void)
{
	PWR->CR1	|= (1<<8);									// Bit 8 DBP: Disable backup domain write protection
	RCC->BDCR |= (1<<0);									// Bit 0 LSEON: LSE oscillator enable
	RCC->BDCR |= (1<<8);									// Bits 9:8 RTCSEL[1:0]: RTC clock source selection
	RCC->BDCR &= ~IO_9;										// Bits 9:8 RTCSEL[1:0]: RTC clock source selection
	RCC->BDCR |= (1<<15);									// Bit 15 RTCEN: RTC clock enable
	
}

void	Systick_EN		(int tick)
{
	SysTick->LOAD = tick-1;
	SysTick->VAL = 0;
	SysTick->CTRL = 7;
}
uint32_t getTick		(void)
{
	__disable_irq();
	_tick = tick;
	__enable_irq();
	
	return _tick;
}
void	wifi_wakeup		(void)
{
	volatile int TimeoutCount = 0;
	volatile int ack = 0;
	char temp = 0;
	
	
	// Initiating the wifi module at startup
	/* goal of this while loop is to recieve welcome message of the wifi module
	// after reset.
	// this loop exits successfully when the message comes with th '>'+SPACE at the
	// end of it. */
	WIFI_RST();
	LED3_ON();		// wifi LED on
	WifiStatus = 1;
	

	while(1){
		wificmdStatus = 0;

		temp = USART3->RDR;
		//_usart1_send_b(temp);		// here we can see the message for debug
		
		if(temp == '>') ack = 1;
		if(ack){
			if(temp == SPACE) goto Done;
		}
		
		++TimeoutCount;
		if(TimeoutCount > 150) goto error;
		while(wificmdStatus == 0);
	}
	
error :
	wificmdStatus = 0;
	nl(1);
	terminal("WiFi Init Error!");// %d", TimeoutCount);
  return;	
	
	
Done :
    //terminal("WiFi OK!");
	wificmdStatus = 0;
	return;
}
//************************************************************************
//* Pheripheral Functions
//************************************************************************

/**
  * @brief  
  * @param  None
  * @retval None
  */
void	wifi_Connect	(void)
{
	int i, j, k,  len = 0;
	int	dbgCnt = 0;
	volatile unsigned int epoch1 = 0, epoch2 = 0;
	uint8_t pData[500];
//	uint8_t cmd01[] = AT_ENTER_CMD_MODE;

	
	//_usart1_printf("start\n\r cmd status %d\n\r", wificmdcount);

	// set of commands to connect to a network


//	_wifi_send("%s\r",AT_ENTER_CMD_MODE);
//	_wifi_send("%s\r",AT_SET_USER_SSID);
//	_wifi_send("%s\r",AT_SET_USER_PASSPHRASE);
//	_wifi_send("%s\r",AT_SET_USER_SECURITY_TYPE);
//	_wifi_send("%s\r",AT_SET_USER_DHCP);
//	_wifi_send("%s\r",AT_NET_JOIN);
//	_wifi_send("HT\r");

//	




}

//************************************************************************
//* System Functions
//************************************************************************
void	OsInits			(void)
{
	__disable_irq();	// Disable interrupts
  
	//Systick_EN(OneMilliSec);
	RCC_Config();
	GPIO_Config();
	USART1_Config();
	USART3_Config();	
	RTC_Config();
	//SPI3_Config();
	ISR_Config();
	
	__enable_irq();
	
	//wifi_wakeup();
	Welcome();
}

void	_wifi_send      (const char *format, ...)
{
	static  uint8_t  buffer[40 + 1];
	va_list     vArgs;
		
	va_start(vArgs, format);
	vsprintf((char *)buffer, (char const *)format, vArgs);
	va_end(vArgs);
	

	_usart3_send_s((char *) buffer);
}
int		_wifi_read      (void)
{
	while(!(USART3->ISR & IO_5));					// RXNE: Read data register not empty
	
	return USART3->RDR;
}


void	delayS			(int Seconds )
{
//	int i;
//	
//	// Because we counting the milliseconds ix1000
//	Seconds *= 1000;
//	i = getTick();
//	
//	while((getTick()- i) < Seconds); 
}
void	delayMs			(int MilliSecond)
{
//	int i;
//	
//	// Because we counting the milliseconds ix1000
//	i = getTick();
//	
//	while((getTick()- i) < MilliSecond); 
}

void	halt			(int num)
{	
	//while(GPIOC->IDR & IO_13)
	nl(1);
	//terminal("Halt_%d", num);
	while(num--)
	{
		//LED1_ON;													// BSRR register low byte is as set
		LED2_OFF();													// BSRR register high byte is as reset
		delayMs(10);

		//LED1_OFF;
		LED2_ON();
		delayMs(10);
	}
	LED1_OFF();
	LED2_OFF();
}

void	Welcome			(void)
{
	int i;
	
	nl(2); 
	_usart1_send_s(wc_note);
	line(2);
	nl(2);
	
	_usart1_send_b(CommandSign);
}


void	nl				(int Count)
{
	int i;
	for (i = 0 ; i < Count ; i++) _usart1_send_s(New_Line);
}
void	line			(int leng)
{
	if(leng == 1){
		nl(1);
		_usart1_send_s("________");
	} else {
		nl(1);
		_usart1_send_s("_______________");
	}
}

void	BckSpc			(uint8_t Line)
{
	uint8_t i;
	
	for(i = 0; i < Line; i++){
		_usart1_send_b(ENTER);
		_usart1_send_b(BACKSPACE1);
	}
	_usart1_send_b(ENTER);
}

void	leds			(void)
{
	GPIOC->BSRR |= IO_4;
	GPIOC->BRR |= IO_5;
	delayMs(30);
	
	GPIOC->BRR |= IO_4;
	GPIOC->BSRR |= IO_5;
	delayMs(30);
	
	GPIOC->BSRR |= IO_4;
	GPIOC->BSRR |= IO_5;
	delayMs(30);
	
	
}
void	led1			(void)
{
	while(1){
		LED1_ON();
		delayMs(300);
		
		LED1_OFF();
		delayMs(300);
	}
}
void	led2			(void)
{
	while(1){
		LED2_ON();
		delayMs(300);
		
		LED2_OFF();
		delayMs(300);
	}
}

void    togglepin       (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  GPIOx->ODR ^= GPIO_Pin;
}

//********************************************f****************************
//* Interrupts
//************************************************************************
void	EXTI1_IRQHandler        (void)
{
	CmdStatus = 5;

	terminal("\n\rim here cmd: %d!", CmdStatus);
	EXTI->PR1 |= EXTI_PR1_PIF1;			// Pending interrupt flag on line x (x = 16 to 0)
	
	
//	nl(1);
}
void	EXTI15_10_IRQHandler	(void)
{
	nl(1);
	_usart1_send_s("Button Pushed!");
	EXTI->PR1 |= EXTI_PR1_PIF13;		// Pending interrupt flag on line x (x = 16 to 0)
	
	nl(1);
	_usart1_send_b(CommandSign);
}
void	USART1_IRQHandler		(void)
{
	CmdStatus = 1;
	USART1->RQR |= (1<<3);
}
void	USART3_IRQHandler		(void)
{
	wificmdStatus = 1;
	USART3->RQR |= (1<<3);
}
//void SysTick_Handler			(void)
//{
//	++tick;
//	 //_usart1_printf("\n\rtick: %d!", tick);
//	// GPIOA->ODR ^= LD1;
//}


//************************************************************************































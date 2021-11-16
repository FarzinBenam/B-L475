/* Includes ------------------------------------------------------------------*/
#include "Configs.h"

/* Exported constants --------------------------------------------------------*/
extern const char	_Error[];
/* Private define ------------------------------------------------------------*/


/**
 *  @brief HTS221 Slave address
 */
static void   bsp_config	(void);
static void		ISR_Config	(void);
static void		rtc_config	(void);
static void		i2c_config	(void);

static void		crc_init8		(void);
static void		crc_init32	(void);



/** @defgroup STM32L475E_IOT01_LOW_LEVEL_Private_FunctionPrototypes LOW LEVEL Private Function Prototypes
  * @{
  */

//************************************************************************
//*	Definitions
//************************************************************************
extern volatile uint32_t tick;


uint8_t Configs (void)
{
  __disable_irq();	// Disable interrupts
  
  bsp_config();
	ISR_Config();
  //_bsp_clk_freq_get();
  i2c_config();
  if(qspi_config() != SUCCESS){
		terminal("\n%sQSPI", _Error);
	}
	rtc_config();
  
  
  
  /* Componentefs Inits  -------------------------------------------------------*/
  HTS221_T_Init(HTS221_SLAVE_ADD);
	HTS221_H_Init(HTS221_SLAVE_ADD);
  
  
  
  __enable_irq();
	nl(4);
	terminal("_____________________\n");
	time_show();
  terminal("\nWelcome! \n_____________________\n");
  return 0;
}

/* ==============   BOARD SPECIFIC CONFIGURATION CODE BEGIN    ============== */
// RCC, GPIO, USART, ISR, SysTick 
static void		bsp_config	(void)
{
  LL_GPIO_InitTypeDef   GPIO_InitDef;
  LL_USART_InitTypeDef  USART_InitStruct;
  
/* RCC Configuration ******************************************************/    
  /* Configure the system clock to 80 MHz */
  /**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = PLL (MSI)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            PLL_M                          = 1
  *            PLL_N                          = 40
  *            PLL_R                          = 2
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
  /* MSI configuration and activation */
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  LL_RCC_MSI_Enable();
  while(LL_RCC_MSI_IsReady() != 1) 
  {
  };
  
  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_MSI, LL_RCC_PLLM_DIV_1, 40, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_Enable();
  LL_RCC_PLL_EnableDomain_SYS();
  while(LL_RCC_PLL_IsReady() != 1) 
  {
  };
  
  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) 
  {
  };
  
  /* Set APB1 & APB2 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  
/* SysTick Configuration **************************************************/    
  #if (systick == 1)
    
    /* Set systick to 1ms in using frequency set to 80MHz */
    /* This frequency can be calculated through LL RCC macro */
    /* ex: __LL_RCC_CALC_PLLCLK_FREQ(__LL_RCC_CALC_MSI_FREQ(LL_RCC_MSIRANGESEL_RUN, LL_RCC_MSIRANGE_6), 
                                    LL_RCC_PLLM_DIV_1, 40, LL_RCC_PLLR_DIV_2)*/
    LL_Init1msTick(80000000);
  #endif
  
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(80000000);
  
  
/* GPIO Configuration ******************************************************/
  
  /* LED1 Pin Config */
  LED1_GPIO_CLK_ENABLE();
  LL_GPIO_StructInit(&GPIO_InitDef);    // de-init the gpio struct
  GPIO_InitDef.Pin = LED1_PIN;
  GPIO_InitDef.Mode = LL_GPIO_MODE_OUTPUT;
  LL_GPIO_Init(LED1_GPIO_PORT, &GPIO_InitDef);
  
  
  
  /* LED2 Pin Config */
  LED2_GPIO_CLK_ENABLE();
  LL_GPIO_StructInit(&GPIO_InitDef);    // de-init the gpio struct
  GPIO_InitDef.Pin = LED2_PIN;
  GPIO_InitDef.Mode = LL_GPIO_MODE_OUTPUT;
  LL_GPIO_Init(LED2_GPIO_PORT, &GPIO_InitDef);
  

  
/* USART Configuration ******************************************************/

  #if (hc_05 == 1)
    /* USART4 RX-TX PINs GPIO init (PA0, PA1) (over HC-05) */  
    GPIOA_CLK_ENABLE()
    
    LL_GPIO_StructInit(&GPIO_InitDef);    // de-init the gpio struct
    GPIO_InitDef.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1;
    GPIO_InitDef.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitDef.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitDef.Alternate = LL_GPIO_AF_8;
    LL_GPIO_Init(GPIOA, &GPIO_InitDef);
    
    
    /* USART1 Clock Enable */
    USART4_CLK_ENABLE();
    
    /* USART4 Struct config */
    LL_USART_StructInit(&USART_InitStruct);
    
    USART_InitStruct.BaudRate = hc_05_baudrate;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    
    /* USART4 Config */
    LL_USART_Init (hc_05_usart, &USART_InitStruct);
    /* USART4 Enable */
    LL_USART_Enable(hc_05_usart);
  #endif

  #if (COM == 1)
    /* USART1 RX-TX PINs GPIO init (PB6=TX, PB7=RX) (over STLINK) */  
    GPIOB_CLK_ENABLE();
    
    LL_GPIO_StructInit(&GPIO_InitDef);    // de-init the gpio struct
    GPIO_InitDef.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
    GPIO_InitDef.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitDef.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitDef.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init(GPIOB, &GPIO_InitDef);
    
    
    
    /* USART1 Clock Enable */
    USART1_CLK_ENABLE();
    
    /* USART1 Struct config */
    LL_USART_StructInit(&USART_InitStruct);
    
    USART_InitStruct.BaudRate = COM_baudrate;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    
    /* USART1 Config */
    LL_USART_Init (COM_usart, &USART_InitStruct);
    /* USART1 Enable */
    LL_USART_Enable(COM_usart);
  #endif 

}

static void		ISR_Config	(void)
{
	RCC->APB2ENR	|= RCC_APB2ENR_SYSCFGEN;// Bit 0 SYSCFGEN: SYSCFG + COMP + VREFBUF clock enable

//	//External Interrupts (EXTI)
//  /*******************************************************************************/  
//	// EXTI for PE1 (WiFi RDY Pin)
//	SYSCFG->EXTICR[0] &= ~(IO_4|IO_5|IO_6|IO_7); 
//	SYSCFG->EXTICR[0] = (1<<6); 
//	EXTI->RTSR1				|= WIFI_RDY_PIN;
//	EXTI->FTSR1				&= ~WIFI_RDY_PIN;
//	EXTI->IMR1				|= WIFI_RDY_PIN;
//	NVIC_SetPriority	(EXTI1_IRQn, 1);			// Priority level 1
//	NVIC_EnableIRQ		(EXTI1_IRQn);
	
	
	//EXTI13 external interrupt
	// select the source input for the EXTI13 external interrupt.
	// EXTICR[3] -> because we count from 0 then EXTICR[3] = EXTICR[4] here
	SYSCFG->EXTICR[3] &= ~(SYSCFG_EXTICR4_EXTI13); 
	SYSCFG->EXTICR[3] = (1<<5); 
	EXTI->RTSR1				&= ~USER_BUTTON_PIN;
	EXTI->FTSR1				|= USER_BUTTON_PIN;
	EXTI->IMR1				|= USER_BUTTON_PIN;
	NVIC_SetPriority	(EXTI15_10_IRQn, 1);	    // Priority level 1
	NVIC_EnableIRQ		(EXTI15_10_IRQn);
	
	
	// USART1 (stlink) or UART4 (HC-05) interrupt
	/*******************************************************************************/  
	// A USART interrupt is generated whenever ORE=1 or RXNE=1 in the USART_ISR register
	terminal_usart->CR1		|= (1 << 5); 			// RXNE interrupt enable
	NVIC_SetPriority	(terminal_IRQn, 1); 		// Priority level 1
	NVIC_EnableIRQ		(terminal_IRQn);
	
	// USART3 interrupt  (ISM43362-M3G-L44 wifi module)
	/*******************************************************************************/  
	// A USART interrupt is generated whenever ORE=1 or RXNE=1 in the USART_ISR register
	USART3->CR1		|= (1 << 5); 					// RXNE interrupt enable
	NVIC_SetPriority	(USART3_IRQn, 1); 			// Priority level 1
	NVIC_EnableIRQ		(USART3_IRQn);
    
	
	/*******************************************************************************/
}
static void		rtc_config	(void)
{
	LL_RTC_InitTypeDef rtc_init;
	
	
	RCC->APB1ENR1 |= (1 << 10); 			// Bit 10 RTCAPBEN: RTC APB clock enable
	RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;				// Bit 28 PWREN: Power interface clock enable
	
  PWR->CR1	|= (1<<8);							// Bit 8 DBP: Disable backup domain write protection
	RCC->BDCR |= RCC_BDCR_LSEON;			// Bit 0 LSEON: LSE oscillator enable
	RCC->BDCR |= RCC_BDCR_RTCSEL_0;		// Bits 9:8 RTCSEL[1:0]: RTC clock source selection
	RCC->BDCR &= ~RCC_BDCR_RTCSEL_1;	// Bits 9:8 RTCSEL[1:0]: RTC clock source selection
	RCC->BDCR |= RCC_BDCR_RTCEN;			// Bit 15 RTCEN: RTC clock enable 
		

	LL_RTC_StructInit(&rtc_init);
	LL_RTC_Init(RTC, &rtc_init);
}

static void   spi_config  (void)
{
  LL_SPI_InitTypeDef *SPI_InitStruct;
  
  LL_SPI_StructInit(SPI_InitStruct);
  
  /* Set SPI_InitStruct fields to default values */
  SPI_InitStruct->TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct->Mode              = LL_SPI_MODE_SLAVE;
  SPI_InitStruct->DataWidth         = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct->ClockPolarity     = LL_SPI_POLARITY_LOW;
  SPI_InitStruct->ClockPhase        = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct->NSS               = LL_SPI_NSS_HARD_INPUT;
  SPI_InitStruct->BaudRate          = LL_SPI_BAUDRATEPRESCALER_DIV2;
  SPI_InitStruct->BitOrder          = LL_SPI_MSB_FIRST;
  SPI_InitStruct->CRCCalculation    = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct->CRCPoly           = 7U;
  
  
}
/* ==============   I2C2 SPECIFI FUNCTIONS         ========================== */
static void		i2c_config (void)
{
  LL_I2C_InitTypeDef    I2C_InitStruct;
  LL_GPIO_InitTypeDef   GPIO_InitDef;
  
  /* (1) Enables GPIO clock and configures the I2C1 pins **********************/
  /*    (SCL = PB10, SDA = PB11)                         **********************/
  
  /* Enable the peripheral clock of GPIOB */
  GPIOB_CLK_ENABLE();
  
  /* Configure Pins as : Alternate function, High Speed, Open drain, Pull up  */
  LL_GPIO_StructInit(&GPIO_InitDef);    // de-init the gpio struct
  
  GPIO_InitDef.Pin = LL_GPIO_PIN_10 | LL_GPIO_PIN_11;
  GPIO_InitDef.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitDef.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitDef.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitDef.Pull = LL_GPIO_PULL_UP;
  GPIO_InitDef.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitDef);
  
  /* (2) Enable the I2C2 peripheral clock *************************************/

  /* Enable the peripheral clock for I2C2 */
  I2C2_CLK_ENABLE();

  /* Set I2C2 clock source as SYSCLK */
  LL_RCC_SetI2CClockSource(LL_RCC_I2C2_CLKSOURCE_SYSCLK);
  
  /* (3) Configure NVIC for I2C2 **********************************************/

//  /* Configure Event IT:
//   *  - Set priority for I2C2_EV_IRQn
//   *  - Enable I2C2_EV_IRQn
//   */
//  NVIC_SetPriority(I2C2_EV_IRQn, 0);  
//  NVIC_EnableIRQ(I2C2_EV_IRQn);

//  /* Configure Error IT:
//   *  - Set priority for I2C2_ER_IRQn
//   *  - Enable I2C2_ER_IRQn
//   */
//  NVIC_SetPriority(I2C2_ER_IRQn, 0);  
//  NVIC_EnableIRQ(I2C2_ER_IRQn);
  
  /* (4) Configure I2C3 functional parameters *********************************/

  /* Disable I2C2 prior modifying configuration registers */
  LL_I2C_Disable(I2C2);

  /* Configure the SDA setup, hold time and the SCL high, low period
   * (uint32_t)0x10909CEC = I2C_TIMING */
  LL_I2C_StructInit(&I2C_InitStruct);
  I2C_InitStruct.Timing = 0x10909CEC;
  LL_I2C_Init(I2C2, &I2C_InitStruct);
  
  /* (5) Enable I2C2 **********************************************************/
  LL_I2C_Enable(I2C2);

  /* (6) Enable I2C2 transfer complete/error interrupts:
   *  - Enable Receive Interrupt
   *  - Enable Not acknowledge received interrupt
   *  - Enable Error interrupts
   *  - Enable Stop interrupt
   */
//  LL_I2C_EnableIT_RX(I2C2);
//  LL_I2C_EnableIT_NACK(I2C2);
//  LL_I2C_EnableIT_ERR(I2C2);
//  LL_I2C_EnableIT_STOP(I2C2);
}


void  i2c2_read									(uint8_t SADD, uint8_t ReadADD, uint32_t TransferSize, uint8_t *buffer)
{
  uint32_t temp = ReadADD;

  
  while((I2C2->ISR & I2C_ISR_BUSY) == SET);
  //terminal("\nbus is not busy, SADD: %X", SADD);
  
//LL_I2C_HandleTransfer(I2Cx, SADD, SlaveAddrSize,         TransferSize, EndMode,             Request)
  LL_I2C_HandleTransfer(I2C2, SADD, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_SOFTEND, LL_I2C_GENERATE_START_WRITE);
  while((I2C2->ISR & I2C_ISR_TXIS) == RESET);
  //terminal("\nSTART + SADD");
  
  
  // In order to read multiple bytes incrementing the register address, it is necessary to assert
  // the most significant bit of the sub-address field.
  if(TransferSize > 1) (temp |= 0x80);
  I2C2->TXDR = temp;                                 // sending memory address of the slave device
  while((I2C2->ISR & I2C_ISR_TC) == RESET);           // wait untl TC flag is set
  //terminal("\nSlave mem Add");
  
  temp = (SADD+1);
//LL_I2C_HandleTransfer(I2Cx, SADD,     SlaveAddrSize,         TransferSize, EndMode,             Request)
  LL_I2C_HandleTransfer(I2C2, temp, LL_I2C_ADDRSLAVE_7BIT, TransferSize, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);
  
  //temp = ReadADD;
  for(int i = 0; i < TransferSize; i++){
    while((I2C2->ISR & I2C_ISR_RXNE) == RESET);
    
    buffer[i] = I2C2->RXDR;
//    terminal("\n%.2X: %X", ReadADD, buffer[i]);
//    ReadADD++;
  }
  
  while((I2C2->ISR & I2C_ISR_STOPF) == RESET);
  //terminal("\nstop sended!");
  I2C2->ICR |= I2C_ICR_STOPCF;      // clearing the STOPF flag
  
  
  
  CLEAR_BIT(I2C2->CR2, 0x3FFFFFF);  // reseting the CR2
/*
 *  LL_I2C_HandleTransfer Macro definition:   
 *MODIFY_REG( I2Cx->CR2,
 *                        I2C_CR2_SADD  |
 *                        I2C_CR2_ADD10 |
 *                        (I2C_CR2_RD_WRN & (uint32_t)(Request >> (31U - I2C_CR2_RD_WRN_Pos))) |
 *                        I2C_CR2_START |
 *                        I2C_CR2_STOP  |
 *                        I2C_CR2_RELOAD|
 *                        I2C_CR2_NBYTES|
 *                        I2C_CR2_AUTOEND |
 *                        I2C_CR2_HEAD10R,
 *                                          SlaveAddr |
 *                                          SlaveAddrSize |
 *                                          (TransferSize << I2C_CR2_NBYTES_Pos) | 
 *                                          EndMode |
 *                                          Request);
*/


  
}

void  i2c2_write								(uint8_t SADD, uint8_t WriteADD, uint32_t TransferSize, uint8_t *buffer)
{  
  while((I2C2->ISR & I2C_ISR_BUSY) == SET);
  //terminal("\nbus is not busy");
  
//LL_I2C_HandleTransfer(I2Cx, SADD, SlaveAddrSize,         TransferSize, EndMode,             Request)
  LL_I2C_HandleTransfer(I2C2, SADD, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_RELOAD, LL_I2C_GENERATE_START_WRITE);
  while((I2C2->ISR & I2C_ISR_TXIS) == RESET);
  //terminal("\nSTART + SADD");
  
  
  // In order to read multiple bytes incrementing the register address, it is necessary to assert
  // the most significant bit of the sub-address field.
  //if(TransferSize > 1) (temp |= 0x80);
  I2C2->TXDR = WriteADD;                              // sending memory address of the slave device
  while((I2C2->ISR & I2C_ISR_TCR) == RESET);          // wait untl TCR flag is set
  //terminal("\nSlave mem Add");
  
  //LL_I2C_HandleTransfer(I2Cx, SADD,     SlaveAddrSize,         TransferSize, EndMode,             Request)
  LL_I2C_HandleTransfer(I2C2, SADD, LL_I2C_ADDRSLAVE_7BIT, TransferSize, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_NOSTARTSTOP);
  
  for(int i = 0; i < TransferSize; i++){
    while((I2C2->ISR & I2C_ISR_TXIS) == RESET);
    
    I2C2->TXDR = buffer[i];
    //terminal("\n%.2X: %X", WriteADD, buffer[i]);
    WriteADD++;
  }
  
  while((I2C2->ISR & I2C_ISR_STOPF) == RESET);
  //terminal("\nstop sended!");
  I2C2->ICR |= I2C_ICR_STOPCF;      // clearing the STOPF flag
  
  
  
  CLEAR_BIT(I2C2->CR2, 0x3FFFFFF);  // reseting the CR2
}








uint8_t i2c2_sensor_read				(uint16_t DeviceAddr, uint8_t RegisterAddr)
{
  uint8_t buffer[1];

  i2c2_read(DeviceAddr, RegisterAddr, 1, buffer);
  
  return buffer[0];
}

void  i2c2_sensor_readmultiple	(uint16_t DeviceAddr, uint8_t RegisterAddr, uint8_t *buffer, uint8_t readsize)
{
  i2c2_read(DeviceAddr, RegisterAddr, readsize, buffer);
}
void  i2c2_sensor_write					(uint16_t DeviceAddr, uint8_t RegisterAddr, uint8_t *tmp)
{
  i2c2_write(DeviceAddr, RegisterAddr, 1, tmp);

}





/* ==============   Time Functions      ===================================== */
void	rtc_set		(LL_RTC_TimeTypeDef *time, LL_RTC_DateTypeDef *date)
{
	LL_RTC_TIME_Init(RTC, LL_RTC_FORMAT_BIN, time);
	LL_RTC_DATE_Init(RTC, LL_RTC_FORMAT_BIN, date);
}
void	rtc_read (LL_RTC_TimeTypeDef	*time, LL_RTC_DateTypeDef	*date)
{
	rtc_time_params timeparams;
	rtc_date_params dateparams;
	uint32_t *ptr;
	
	
	ptr = &timeparams;	/* store address of timeparam in pointer variable*/
	*ptr = RTC->TR;			/* store the value to the pointing address */
	
	time->Seconds = timeparams.second;
	time->Minutes = timeparams.minute;
	time->Hours = timeparams.hour;
	
	ptr = &dateparams; /* store address of dateparam in pointer variable*/
	*ptr = RTC->DR;
	
	date->Day = dateparams.Day;
	date->Month = dateparams.month;
	date->WeekDay = dateparams.weekday;
	date->Year = dateparams.year;
}






/* ==============   CRC Functions      ====================================== */
static void	crc_init8		(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
	
	CRC->CR  |= CRC_CR_RESET;             // Reset calculation
	CRC->POL  = 0xCB;                     // pick a random poly
	CRC->CR   = 2 << CRC_CR_POLYSIZE_Pos;	// set poly to 8 bit
	CRC->INIT = 0xFF; 
}
static void	crc_init32	(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;

	CRC->CR  |= CRC_CR_RESET;							// Reset calculation
	CRC->POL  = 0x04C11DB7U;							// Default 32 bit poly
	CRC->CR   = 0x00000000;       				// sets poly size to 32 bit
	CRC->INIT = 0xFFFFFFFF;								//32 bit init value
}

uint8_t	crc_8bit				(uint8_t * buffer, uint8_t length)
{
	volatile uint8_t i, temp;
	
	crc_init8();
	
	for(i = 0; i < length; i++){
		CRC->DR = buffer[i];
	}
	temp = CRC->DR;
	
	RCC->AHB1ENR &= ~RCC_AHB1ENR_CRCEN;
	return temp;
}

/* ==============   General Functions  ====================================== */
void  _bsp_clk_freq_get (void)
{
  LL_RCC_ClocksTypeDef RCC_Clocks;
  
  LL_RCC_GetSystemClocksFreq(&RCC_Clocks);
  
  
  terminal("\nSYSCLK:%d", RCC_Clocks.HCLK_Frequency);
  terminal("\nHCLK:  %d", RCC_Clocks.HCLK_Frequency);
  terminal("\nPCLK1: %d", RCC_Clocks.PCLK1_Frequency);
  terminal("\nPCLK2: %d", RCC_Clocks.PCLK2_Frequency);
  
}

void	nl	(uint8_t line)
{
	uint8_t i;
	for(i = 0; i < line; i++){
		terminal("\n");
	}
}
void  halt  (char Message[]){
  terminal("\n______________HALT\n%s", Message);
  while(1); 
}
/* ========================================================================== */

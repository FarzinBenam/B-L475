/**
  * @file			
  * @author		Farzin M.Benam
  * @version	
  * @date     2021-08-23
  * @brief		
  *             
  *******************************************************************************

  *******************************************************************************/ 
#include "qspi.h"

/* Private define ------------------------------------------------------------*/
/** @defgroup QSPI_Private_Constants QSPI Private Constants
  * @{
  */
#define MEMORY_SECTOR_SIZE                  MX25R6435F_SECTOR_SIZE
#define MEMORY_PAGE_SIZE                    MX25R6435F_PAGE_SIZE


#define QSPI_FUNCTIONAL_MODE_INDIRECT_WRITE 0x00000000U                     /*!<Indirect write mode*/
#define QSPI_FUNCTIONAL_MODE_INDIRECT_READ  ((uint32_t)QUADSPI_CCR_FMODE_0) /*!<Indirect read mode*/
#define QSPI_FUNCTIONAL_MODE_AUTO_POLLING   ((uint32_t)QUADSPI_CCR_FMODE_1) /*!<Automatic polling mode*/
#define QSPI_FUNCTIONAL_MODE_MEMORY_MAPPED  ((uint32_t)QUADSPI_CCR_FMODE)   /*!<Memory-mapped mode*/
/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
static void MX_QUADSPI_Init (void);
static void qspi_ResetChip (void);
static void QSPI_Config(QSPI_CommandTypeDef *cmd, uint32_t FunctionalMode);
static void qspi_gpio_Init (void);
static void mx_qspi_Init (void);
static void qspi_Configuration (void);
static void qspi_AutoPollingMemReady (void);
static void qspi_AutoPolling (QSPI_CommandTypeDef *cmd, QSPI_AutoPollingTypeDef *cfg);
static void qspi_Read_All_Reg (uint8_t* test_buffer);
static void qspi_Abort (void);
static void qspi_WriteEnable(void);
static void qspi_Command (QSPI_CommandTypeDef *cmd);
static void qspi_Transmit (uint8_t *pData);
static void qspi_Receive (uint8_t *pData);

/*******************************************************************************
 * QUADSPI Functions
 *******************************************************************************/
 /* QUADSPI init function */
void  qspi_config (void)
{
  /* (1) Enables GPIO clock and confige the QPSI pins ************************/
  qspi_gpio_Init ();
  /* (2) Enable the QSPI peripheral clock *************************************/
	mx_qspi_Init();
  /* (3) QSPI_ResetChip session  **********************************************/
  qspi_ResetChip();
  
  LL_mDelay(10);
  
  qspi_AutoPollingMemReady();

	qspi_WriteEnable();

	qspi_Configuration();
}

/**
  * @brief Set the command configuration.
  * @param cmd : structure that contains the command configuration information
  * @note   This function is used only in Indirect Read or Write Modes
  */
void  qspi_Command(QSPI_CommandTypeDef *cmd)
{
  /* Wait till BUSY flag reset */
  while(QUADSPI->SR & QUADSPI_SR_BUSY) { }
  
  /* Call the configuration function */
  QSPI_Config(cmd, QSPI_FUNCTIONAL_MODE_INDIRECT_WRITE);

  if (cmd->DataMode == QSPI_DATA_NONE)
  {
    /* When there is no data phase, the transfer start as soon as the configuration is done
    so wait until TC flag is set to go back in idle state */
    while((QUADSPI->SR & QUADSPI_SR_TCF) == RESET) { }
    
    /* Clears QSPI_FLAG_TC's flag status. */
    QUADSPI->FCR |= QUADSPI_FCR_CTCF;
  }
}

/**
  * @brief Transmit an amount of data in blocking mode.
  * @param hqspi : QSPI handle
  * @param pData : pointer to data buffer
  * @param Timeout : Timeout duration
  * @note   This function is used only in Indirect Write Mode
  * @retval HAL status
  */
void  qspi_Transmit (uint8_t *pData)
{
  __IO uint32_t *data_reg = &QUADSPI->DR;
  __IO uint32_t              TxXferSize;       /* QSPI Tx Transfer size              */
  __IO uint32_t              TxXferCount;      /* QSPI Tx Transfer Counter           */
  uint8_t                    *pTxBuffPtr;      /* Pointer to QSPI Tx transfer Buffer */

  if(pData != NULL )
  {
    

    /* Configure counters and size of the handle */
    TxXferCount = READ_REG(QUADSPI->DLR) + 1U;
    TxXferSize = READ_REG(QUADSPI->DLR) + 1U;
    pTxBuffPtr = pData;

    /* Configure QSPI: CCR register with functional as indirect write */
    MODIFY_REG(QUADSPI->CCR, QUADSPI_CCR_FMODE, QSPI_FUNCTIONAL_MODE_INDIRECT_WRITE);

    while(TxXferCount > 0U)
    {
      /* Wait until FT flag is set to send data */
      while((QUADSPI->SR & QUADSPI_SR_FTF) == RESET);

      *((__IO uint8_t *)data_reg) = *pTxBuffPtr;
      pTxBuffPtr++;
      TxXferCount--;
    }
    
    /* Wait until TC flag is set to go back in idle state */
    while((QUADSPI->SR & QUADSPI_SR_TCF) == RESET);
    
    /* Clear Transfer Complete bit */
    QUADSPI->FCR |= QUADSPI_FCR_CTCF;

  #if  (defined(STM32L471xx) || defined(STM32L475xx) || defined(STM32L476xx) || defined(STM32L485xx) || defined(STM32L486xx))
    /* Clear Busy bit */
    qspi_Abort();
  #endif
    }
}


/**
  * @brief Receive an amount of data in blocking mode.
  * @param hqspi : QSPI handle
  * @param pData : pointer to data buffer
  * @param Timeout : Timeout duration
  * @note   This function is used only in Indirect Read Mode
  * @retval HAL status
  */
void  qspi_Receive (uint8_t *pData)
{
  uint32_t addr_reg = READ_REG(QUADSPI->AR);
  __IO uint32_t *data_reg = &QUADSPI->DR;
  uint8_t                    *pRxBuffPtr;      /* Pointer to QSPI Rx transfer Buffer */
  __IO uint32_t              RxXferSize;       /* QSPI Rx Transfer size              */
  __IO uint32_t              RxXferCount;      /* QSPI Rx Transfer Counter           */

  if(pData != NULL )
  {
    /* Configure counters and size of the handle */
    RxXferCount = READ_REG(QUADSPI->DLR) + 1U;
    RxXferSize = READ_REG(QUADSPI->DLR) + 1U;
    pRxBuffPtr = pData;

    /* Configure QSPI: CCR register with functional as indirect read */
    MODIFY_REG(QUADSPI->CCR, QUADSPI_CCR_FMODE, QSPI_FUNCTIONAL_MODE_INDIRECT_READ);

    /* Start the transfer by re-writing the address in AR register */
    WRITE_REG(QUADSPI->AR, addr_reg);

    while(RxXferCount > 0U)
    {
      /* Wait until FT or TC flag is set to read received data */
      while((QUADSPI->SR & (QUADSPI_SR_TCF|QUADSPI_SR_FTF)) == RESET);

      

      *pRxBuffPtr = *((__IO uint8_t *)data_reg);
      pRxBuffPtr++;
      RxXferCount--;
    }


    /* Wait until TC flag is set to go back in idle state */
    while((QUADSPI->SR & QUADSPI_SR_TCF) == RESET);
    /* Clear Transfer Complete bit */
    QUADSPI->FCR |= QUADSPI_FCR_CTCF;
    
    #if  (defined(STM32L471xx) || defined(STM32L475xx) || defined(STM32L476xx) || defined(STM32L485xx) || defined(STM32L486xx))
      /* Workaround - Extra data written in the FIFO at the end of a read transfer */
      qspi_Abort();
    #endif
  }
}


void  qspi_EraseSector(uint32_t EraseStartAddress, uint32_t EraseEndAddress)
{

  QSPI_CommandTypeDef sCommand;

	EraseStartAddress = EraseStartAddress - EraseStartAddress % MEMORY_SECTOR_SIZE;

	/* Erasing Sequence -------------------------------------------------- */
	sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	sCommand.AddressSize = QSPI_ADDRESS_24_BITS;
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
	sCommand.Instruction = SECTOR_ERASE_CMD;
	sCommand.AddressMode = QSPI_ADDRESS_1_LINE;

	sCommand.DataMode = QSPI_DATA_NONE;
	sCommand.DummyCycles = 0;

	while (EraseEndAddress >= EraseStartAddress) {
		sCommand.Address = (EraseStartAddress & 0x0FFFFFFF);

		qspi_WriteEnable();

		qspi_Command(&sCommand);
		EraseStartAddress += MEMORY_SECTOR_SIZE;

		qspi_AutoPollingMemReady();
	}
}

void  qspi_WriteMemory(uint8_t* buffer, uint32_t address,uint32_t buffer_size)
{

	QSPI_CommandTypeDef sCommand;
	uint32_t end_addr, current_size, current_addr;

	/* Calculation of the size between the write address and the end of the page */
	current_addr = 0;


	//
	while (current_addr <= address) {
		current_addr += MEMORY_PAGE_SIZE;
	}
	current_size = current_addr - address;

	/* Check if the size of the data is less than the remaining place in the page */
	if (current_size > buffer_size) {
		current_size = buffer_size;
	}

	/* Initialize the adress variables */
	current_addr = address;
	end_addr = address + buffer_size;

	sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	sCommand.AddressSize = QSPI_ADDRESS_24_BITS;
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
#ifdef ONE_LINE_WRITE
	// one line
	sCommand.Instruction = PAGE_PROG_CMD;
	sCommand.AddressMode = QSPI_ADDRESS_1_LINE;
	sCommand.DataMode = QSPI_DATA_1_LINE;
#else
	// four lines
	sCommand.Instruction = QUAD_PAGE_PROG_CMD;
	sCommand.AddressMode = QSPI_ADDRESS_4_LINES;
	sCommand.DataMode = QSPI_DATA_4_LINES;
#endif
	sCommand.NbData = buffer_size;
	sCommand.Address = address;
	sCommand.DummyCycles = 0;

	/* Perform the write page by page */
	do {
		sCommand.Address = current_addr;
		sCommand.NbData = current_size;

		if (current_size == 0) {
			break;
		}

		/* Enable write operations */
		qspi_WriteEnable();

		/* Configure the command */
		qspi_Command(&sCommand);

		/* Transmission of the data */
		qspi_Transmit(buffer);

		/* Configure automatic polling mode to wait for end of program */
		qspi_AutoPollingMemReady();

		/* Update the address and size variables for next page programming */
		current_addr += current_size;
		buffer += current_size;
		current_size =
				((current_addr + MEMORY_PAGE_SIZE) > end_addr) ?
						(end_addr - current_addr) : MEMORY_PAGE_SIZE;
	} while (current_addr <= end_addr);

}

void  qspi_ReadMemory(uint8_t* buffer, uint32_t address,uint32_t buffer_size)
{
  QSPI_CommandTypeDef sCommand;
	sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	sCommand.AddressSize = QSPI_ADDRESS_24_BITS;
	sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
	sCommand.AddressMode = QSPI_ADDRESS_4_LINES;
	sCommand.DataMode = QSPI_DATA_4_LINES;
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_4_LINES;
	sCommand.AlternateBytes = MX25R6435F_ALT_BYTES_PE_MODE;
	sCommand.AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
	sCommand.NbData = buffer_size;
	sCommand.Address = address;
	sCommand.Instruction = QUAD_INOUT_READ_CMD;
	sCommand.DummyCycles = MX25R6435F_DUMMY_CYCLES_READ_QUAD;


  /* Configure the command */
  qspi_Command(&sCommand);

  /* Transmission of the data */
  qspi_Receive(buffer);

  uint8_t j[1] = {0};
  sCommand.AlternateBytes = MX25R6435F_ALT_BYTES_NO_PE_MODE;
  sCommand.NbData = 1;
  /* Configure the command */
  qspi_Command(&sCommand);

  /* Transmission of the data */
  qspi_Receive(j);
}
void  qspi_Erase_Chip(void)
{
	QSPI_CommandTypeDef sCommand;

	qspi_WriteEnable();

	/* Erasing Sequence --------------------------------- */
	sCommand.Instruction = CHIP_ERASE_CMD;
	sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	sCommand.AddressSize = QSPI_ADDRESS_24_BITS;
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
	sCommand.AddressMode = QSPI_ADDRESS_NONE;
	sCommand.Address = 0;
	sCommand.DataMode = QSPI_DATA_NONE;
	sCommand.DummyCycles = 0;


	qspi_Command(&sCommand);

	qspi_AutoPollingMemReady();
  terminal("\nChip Erase: OK!");
}

/*******************************************************************************
 * QUADSPI static Functions
 *******************************************************************************/
static void qspi_ResetChip (void)
{
  
	QSPI_CommandTypeDef qspiCmd;
	uint32_t temp = 0;
	/* Erasing Sequence -------------------------------------------------- */
	qspiCmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	qspiCmd.AddressSize       = QSPI_ADDRESS_24_BITS;
	qspiCmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	qspiCmd.DdrMode           = QSPI_DDR_MODE_DISABLE;
	qspiCmd.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	qspiCmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
	qspiCmd.Instruction       = RESET_ENABLE_CMD;
	qspiCmd.AddressMode       = QSPI_ADDRESS_NONE;
	qspiCmd.Address           = 0;
	qspiCmd.DataMode          = QSPI_DATA_NONE;
	qspiCmd.DummyCycles       = 0;

	qspi_Command(&qspiCmd);
  
	for (temp = 0; temp < 0x2f; temp++) {
		__NOP();
	}

	qspiCmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	qspiCmd.AddressSize = QSPI_ADDRESS_24_BITS;
	qspiCmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	qspiCmd.DdrMode = QSPI_DDR_MODE_DISABLE;
	qspiCmd.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	qspiCmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
	qspiCmd.Instruction = RESET_MEMORY_CMD;
	qspiCmd.AddressMode = QSPI_ADDRESS_NONE;
	qspiCmd.Address = 0;
	qspiCmd.DataMode = QSPI_DATA_NONE;
	qspiCmd.DummyCycles = 0;

	qspi_Command(&qspiCmd);
}


/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void mx_qspi_Init (void)
{
  uint32_t temp;
  volatile int i = 0;
  
  /* Enable the Quad-SPI interface clock */
  RCC->AHB3ENR |= RCC_AHB3ENR_QSPIEN;
  /* Reset QSPI peripheral */
  RCC->AHB3RSTR |= RCC_AHB3RSTR_QSPIRST;  // Reset
  RCC->AHB3RSTR &= ~RCC_AHB3RSTR_QSPIRST; // Release reset

  /*
    QUADSPI Initialization:
  - ClockPrescaler = 0, QSPI clock = FAHB / 1 = 80MHz / 1 = 80MHz
  - FIFO when 8 more bytes written or read
  - don't sample the data read from memory half-clock cycle later
  - flash size = 64Mb = 8MB = 2^(22+1) bytes. 
  - the read and wirte command should CS# high in 30ns
  - clock stay low bwteen two command
  */
  
  /* configure the clock prescaller,
                   fifo threshold, 
                   the clock mode, 
                   sample shifting.
  */
  QUADSPI->CR |= (((4 - 1U) << QUADSPI_CR_FTHRES_Pos)|
                  (0 << QUADSPI_CR_PRESCALER_Pos)|
                  (0 << QUADSPI_CR_SSHIFT));
  
  /* Wait till BUSY flag reset */
  while(QUADSPI->SR & QUADSPI_SR_BUSY){ }
  
  /* Configure QSPI Flash Size, CS High Time and Clock Mode */
  i = 0;
  temp = MX25R6435F_FLASH_SIZE;
  do{             // finding the flash size 2 ^ (x+1) = size
    temp /= 2;
    i++;
  }while(temp > 2);
  QUADSPI->DCR |= ((i << QUADSPI_DCR_FSIZE_Pos)|
                   (0 << QUADSPI_DCR_CSHT_Pos));
  
  /* Enable the peripheral. */
  QUADSPI->CR |= QUADSPI_CR_EN;
  
  #if (qspi_debug == 1)
    terminal("\nCR : %X", QUADSPI->CR);
    terminal("\nDCR: %X", QUADSPI->DCR);
    terminal("\nSR : %X", QUADSPI->SR);
    terminal("\nFCR: %X", QUADSPI->FCR);
    terminal("\nDLR: %X", QUADSPI->DLR);
    terminal("\nCCR: %X", QUADSPI->CCR);
    terminal("\nAR : %X", QUADSPI->AR);
    terminal("\nABR: %X", QUADSPI->ABR);
    terminal("\nDR : %X", QUADSPI->DR);
    terminal("\nPSMKR: %X", QUADSPI->PSMKR);
    terminal("\nPSMAR: %X", QUADSPI->PSMAR);
    terminal("\nPIR: %X", QUADSPI->PIR);
    terminal("\nLPTR: %X", QUADSPI->LPTR);
    terminal("\n______________");
  #endif
}
static void qspi_gpio_Init (void)
{
  LL_GPIO_InitTypeDef GPIO_InitDef;
  
  /*    (SCL = PB10, SDA = PB11) 
  PE10 - QUADSPI_CLK
  PE11 - QUADSPI_NCS
  PE12 - QUADSPI_BK1_IO0
  PE13 - QUADSPI_BK1_IO1
  PE14 - QUADSPI_BK1_IO2
  PE15 - QUADSPI_BK1_IO3
  */
  /* Enable the peripheral clock of GPIOE */
  GPIOE_CLK_ENABLE();
  
  /* Configure Pins as :  Alternate function (AF10),
                          High Speed,
                          Push-Pull,
                          No pull-up or pull-down
  */
  LL_GPIO_StructInit(&GPIO_InitDef);    // de-init the gpio struct
  
  GPIO_InitDef.Pin = LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15;
  GPIO_InitDef.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitDef.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitDef.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitDef.Pull = LL_GPIO_PULL_NO;
  GPIO_InitDef.Alternate = LL_GPIO_AF_10;
  LL_GPIO_Init(GPIOE, &GPIO_InitDef);
}


static void qspi_AutoPolling (QSPI_CommandTypeDef *cmd, QSPI_AutoPollingTypeDef *cfg)
{
  /* Wait till BUSY flag reset */
  while(QUADSPI->SR & QUADSPI_SR_BUSY) { }

  /* Configure QSPI: PSMAR register with the status match value */
  WRITE_REG(QUADSPI->PSMAR, cfg->Match);

  /* Configure QSPI: PSMKR register with the status mask value */
  WRITE_REG(QUADSPI->PSMKR, cfg->Mask);

  /* Configure QSPI: PIR register with the interval value */
  WRITE_REG(QUADSPI->PIR, cfg->Interval);

  /* Configure QSPI: CR register with Match mode and Automatic stop enabled
  (otherwise there will be an infinite loop in blocking mode) */
  MODIFY_REG(QUADSPI->CR, (QUADSPI_CR_PMM | QUADSPI_CR_APMS),
           (cfg->MatchMode | QSPI_AUTOMATIC_STOP_ENABLE));

  /* Call the configuration function */
  cmd->NbData = cfg->StatusBytesSize;
  QSPI_Config(cmd, QSPI_FUNCTIONAL_MODE_AUTO_POLLING);

  /* Wait until SM flag is set to go back in idle state */
  while((QUADSPI->SR & QUADSPI_SR_SMF) == RESET) { }


  /* Clears QSPI_FLAG_SM's flag status. */
  QUADSPI->FCR |= QUADSPI_FCR_CSMF;
}
static void qspi_AutoPollingMemReady (void)
{
  QSPI_CommandTypeDef qspiCmd;
	QSPI_AutoPollingTypeDef qspiCfg;

	/* Configure automatic polling mode to wait for memory ready ------ */
	qspiCmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	qspiCmd.Instruction = READ_STATUS_REG_CMD;
	qspiCmd.AddressMode = QSPI_ADDRESS_NONE;
	qspiCmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	qspiCmd.DataMode = QSPI_DATA_1_LINE;
	qspiCmd.DummyCycles = 0;
	qspiCmd.DdrMode = QSPI_DDR_MODE_DISABLE;
	qspiCmd.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	qspiCmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

	qspiCfg.Match = 0x40;
	qspiCfg.Mask = 0xFF;
	qspiCfg.MatchMode = QSPI_MATCH_MODE_AND;
	qspiCfg.StatusBytesSize = 1;
	qspiCfg.Interval = 0x10;
	qspiCfg.AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE;

  qspi_AutoPolling(&qspiCmd , &qspiCfg);
}
static void qspi_WriteEnable (void)
{
	QSPI_CommandTypeDef qspiCmd;
	QSPI_AutoPollingTypeDef sConfig;

	/* Enable write operations ------------------------------------------ */
	qspiCmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	qspiCmd.Instruction = WRITE_ENABLE_CMD;
	qspiCmd.AddressMode = QSPI_ADDRESS_NONE;
	qspiCmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	qspiCmd.DataMode = QSPI_DATA_NONE;
	qspiCmd.DummyCycles = 0;
	qspiCmd.DdrMode = QSPI_DDR_MODE_DISABLE;
	qspiCmd.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	qspiCmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  qspi_Command(&qspiCmd);

	/* Configure automatic polling mode to wait for write enabling ---- */
	sConfig.Match = 0x02;
	sConfig.Mask = 0x02;
	sConfig.MatchMode = QSPI_MATCH_MODE_AND;
	sConfig.StatusBytesSize = 1;
	sConfig.Interval = 0x10;
	sConfig.AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE;

	qspiCmd.Instruction = READ_STATUS_REG_CMD;
	qspiCmd.DataMode = QSPI_DATA_1_LINE;
  
	qspi_AutoPolling(&qspiCmd, &sConfig);
}
/*
 Enable quad mode and set dummy cycles count
*/
static void qspi_Configuration (void)
{
	QSPI_CommandTypeDef qspiCmd;
	uint8_t test_buffer[4] = { 0 };
	qspi_Read_All_Reg(test_buffer);
	/*modify buffer to enable quad mode*/
	test_buffer[0] |= 0x40;

	/*set dummy cycles*/
	test_buffer[1] &= ~0xC0;

	/*enable hight proform*/
	test_buffer[2] |= 0x02;

	qspiCmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	qspiCmd.AddressSize = QSPI_ADDRESS_24_BITS;
	qspiCmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	qspiCmd.DdrMode = QSPI_DDR_MODE_DISABLE;
	qspiCmd.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	qspiCmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
	qspiCmd.Instruction = WRITE_STATUS_CFG_REG_CMD;
	qspiCmd.AddressMode = QSPI_ADDRESS_NONE;
	qspiCmd.DataMode = QSPI_DATA_1_LINE;
	qspiCmd.DummyCycles = 0;
	qspiCmd.NbData = 3;

	qspi_Command(&qspiCmd);

	qspi_Transmit(test_buffer);
}
static void qspi_Read_All_Reg (uint8_t* test_buffer)
{
	QSPI_CommandTypeDef qspiCmd;
	/*read status register*/
	qspiCmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	qspiCmd.Instruction = READ_STATUS_REG_CMD;
	qspiCmd.AddressMode = QSPI_ADDRESS_NONE;
	qspiCmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	qspiCmd.DataMode = QSPI_DATA_1_LINE;
	qspiCmd.DummyCycles = 0;
	qspiCmd.DdrMode = QSPI_DDR_MODE_DISABLE;
	qspiCmd.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	qspiCmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
	qspiCmd.NbData = 1;

	qspi_Command(&qspiCmd);
  qspi_Receive(test_buffer);
  
	/*read configuration register*/
	qspiCmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	qspiCmd.Instruction = READ_CFG_REG_CMD;
	qspiCmd.AddressMode = QSPI_ADDRESS_NONE;
	qspiCmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	qspiCmd.DataMode = QSPI_DATA_1_LINE;
	qspiCmd.DummyCycles = 0;
	qspiCmd.DdrMode = QSPI_DDR_MODE_DISABLE;
	qspiCmd.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	qspiCmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
	qspiCmd.NbData = 2;

	qspi_Command(&qspiCmd);
	qspi_Receive(&(test_buffer[1]));
  
}
/**
* @brief  Abort the current transmission.
* @param  hqspi : QSPI handle
* @retval HAL status
*/
static void qspi_Abort (void)
{
  if ((QUADSPI->CR & QUADSPI_CR_DMAEN) != 0U)
  {
    /* Disable the DMA transfer by clearing the DMAEN bit in the QSPI CR register */
    CLEAR_BIT(QUADSPI->CR, QUADSPI_CR_DMAEN);
  }

  /* Configure QSPI: CR register with Abort request */
  SET_BIT(QUADSPI->CR, QUADSPI_CR_ABORT);

  /* Wait until TC flag is set to go back in idle state */
  while((QUADSPI->SR & QUADSPI_SR_TCF) == RESET) { }

  QUADSPI->FCR |= QUADSPI_FCR_CTCF;


  /* Wait until BUSY flag is reset */
  while((QUADSPI->SR & QUADSPI_SR_TCF) == SET) { }

  /* Reset functional mode configuration to indirect write mode by default */
  CLEAR_BIT(QUADSPI->CCR, QUADSPI_CCR_FMODE);
}

/**
  * @brief  Configure the communication registers.
  * @param  cmd : structure that contains the command configuration information
  * @param  FunctionalMode : functional mode to configured
  *           This parameter can be one of the following values:
  *            @arg QSPI_FUNCTIONAL_MODE_INDIRECT_WRITE: Indirect write mode
  *            @arg QSPI_FUNCTIONAL_MODE_INDIRECT_READ: Indirect read mode
  *            @arg QSPI_FUNCTIONAL_MODE_AUTO_POLLING: Automatic polling mode
  *            @arg QSPI_FUNCTIONAL_MODE_MEMORY_MAPPED: Memory-mapped mode
  * @retval None
  */
static void QSPI_Config(QSPI_CommandTypeDef *cmd, uint32_t FunctionalMode)
{

  if ((cmd->DataMode != QSPI_DATA_NONE) && (FunctionalMode != QSPI_FUNCTIONAL_MODE_MEMORY_MAPPED))
  {
    /* Configure QSPI: DLR register with the number of data to read or write */
    WRITE_REG(QUADSPI->DLR, (cmd->NbData - 1U));
  }

  if (cmd->InstructionMode != QSPI_INSTRUCTION_NONE)
  {
    if (cmd->AlternateByteMode != QSPI_ALTERNATE_BYTES_NONE)
    {
      /* Configure QSPI: ABR register with alternate bytes value */
      WRITE_REG(QUADSPI->ABR, cmd->AlternateBytes);

      if (cmd->AddressMode != QSPI_ADDRESS_NONE)
      {
        /*---- Command with instruction, address and alternate bytes ----*/
        /* Configure QSPI: CCR register with all communications parameters */
        WRITE_REG(QUADSPI->CCR, (cmd->DdrMode | cmd->DdrHoldHalfCycle | cmd->SIOOMode |
                                         cmd->DataMode | (cmd->DummyCycles << QUADSPI_CCR_DCYC_Pos) |
                                         cmd->AlternateBytesSize | cmd->AlternateByteMode |
                                         cmd->AddressSize | cmd->AddressMode | cmd->InstructionMode |
                                         cmd->Instruction | FunctionalMode));

        if (FunctionalMode != QSPI_FUNCTIONAL_MODE_MEMORY_MAPPED)
        {
          /* Configure QSPI: AR register with address value */
          WRITE_REG(QUADSPI->AR, cmd->Address);
        }
      }
      else
      {
        /*---- Command with instruction and alternate bytes ----*/
        /* Configure QSPI: CCR register with all communications parameters */
        WRITE_REG(QUADSPI->CCR, (cmd->DdrMode | cmd->DdrHoldHalfCycle | cmd->SIOOMode |
                                         cmd->DataMode | (cmd->DummyCycles << QUADSPI_CCR_DCYC_Pos) |
                                         cmd->AlternateBytesSize | cmd->AlternateByteMode |
                                         cmd->AddressMode | cmd->InstructionMode |
                                         cmd->Instruction | FunctionalMode));
      }
    }
    else
    {
      if (cmd->AddressMode != QSPI_ADDRESS_NONE)
      {
        /*---- Command with instruction and address ----*/
        /* Configure QSPI: CCR register with all communications parameters */
        WRITE_REG(QUADSPI->CCR, (cmd->DdrMode | cmd->DdrHoldHalfCycle | cmd->SIOOMode |
                                         cmd->DataMode | (cmd->DummyCycles << QUADSPI_CCR_DCYC_Pos) |
                                         cmd->AlternateByteMode | cmd->AddressSize | cmd->AddressMode |
                                         cmd->InstructionMode | cmd->Instruction | FunctionalMode));

        if (FunctionalMode != QSPI_FUNCTIONAL_MODE_MEMORY_MAPPED)
        {
          /* Configure QSPI: AR register with address value */
          WRITE_REG(QUADSPI->AR, cmd->Address);
        }
      }
      else
      {
        /*---- Command with only instruction ----*/
        /* Configure QSPI: CCR register with all communications parameters */
        WRITE_REG(QUADSPI->CCR, (cmd->DdrMode | cmd->DdrHoldHalfCycle | cmd->SIOOMode |
                                         cmd->DataMode | (cmd->DummyCycles << QUADSPI_CCR_DCYC_Pos) |
                                         cmd->AlternateByteMode | cmd->AddressMode |
                                         cmd->InstructionMode | cmd->Instruction | FunctionalMode));
      }
    }
  }
  else
  {
    if (cmd->AlternateByteMode != QSPI_ALTERNATE_BYTES_NONE)
    {
      /* Configure QSPI: ABR register with alternate bytes value */
      WRITE_REG(QUADSPI->ABR, cmd->AlternateBytes);

      if (cmd->AddressMode != QSPI_ADDRESS_NONE)
      {
        /*---- Command with address and alternate bytes ----*/
        /* Configure QSPI: CCR register with all communications parameters */
        WRITE_REG(QUADSPI->CCR, (cmd->DdrMode | cmd->DdrHoldHalfCycle | cmd->SIOOMode |
                                         cmd->DataMode | (cmd->DummyCycles << QUADSPI_CCR_DCYC_Pos) |
                                         cmd->AlternateBytesSize | cmd->AlternateByteMode |
                                         cmd->AddressSize | cmd->AddressMode |
                                         cmd->InstructionMode | FunctionalMode));

        if (FunctionalMode != QSPI_FUNCTIONAL_MODE_MEMORY_MAPPED)
        {
          /* Configure QSPI: AR register with address value */
          WRITE_REG(QUADSPI->AR, cmd->Address);
        }
      }
      else
      {
        /*---- Command with only alternate bytes ----*/
        /* Configure QSPI: CCR register with all communications parameters */
        WRITE_REG(QUADSPI->CCR, (cmd->DdrMode | cmd->DdrHoldHalfCycle | cmd->SIOOMode |
                                         cmd->DataMode | (cmd->DummyCycles << QUADSPI_CCR_DCYC_Pos) |
                                         cmd->AlternateBytesSize | cmd->AlternateByteMode |
                                         cmd->AddressMode | cmd->InstructionMode | FunctionalMode));
      }
    }
    else
    {
      if (cmd->AddressMode != QSPI_ADDRESS_NONE)
      {
        /*---- Command with only address ----*/
        /* Configure QSPI: CCR register with all communications parameters */
        WRITE_REG(QUADSPI->CCR, (cmd->DdrMode | cmd->DdrHoldHalfCycle | cmd->SIOOMode |
                                         cmd->DataMode | (cmd->DummyCycles << QUADSPI_CCR_DCYC_Pos) |
                                         cmd->AlternateByteMode | cmd->AddressSize |
                                         cmd->AddressMode | cmd->InstructionMode | FunctionalMode));

        if (FunctionalMode != QSPI_FUNCTIONAL_MODE_MEMORY_MAPPED)
        {
          /* Configure QSPI: AR register with address value */
          WRITE_REG(QUADSPI->AR, cmd->Address);
        }
      }
      else
      {
        /*---- Command with only data phase ----*/
        if (cmd->DataMode != QSPI_DATA_NONE)
        {
          /* Configure QSPI: CCR register with all communications parameters */
          WRITE_REG(QUADSPI->CCR, (cmd->DdrMode | cmd->DdrHoldHalfCycle | cmd->SIOOMode |
                                           cmd->DataMode | (cmd->DummyCycles << QUADSPI_CCR_DCYC_Pos) |
                                           cmd->AlternateByteMode | cmd->AddressMode |
                                           cmd->InstructionMode | FunctionalMode));
        }
      }
    }
  }
}



/******************* (C) COPYRIGHT 2021 Farzin_M.Benam *****END OF FILE****/

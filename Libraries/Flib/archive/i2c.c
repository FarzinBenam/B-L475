#include "i2c.h"

#define DevAddress 0xBE


void i2c_config (I2C_TypeDef *i2cx)
{
  uint32_t Timing = 0x00000E14;
  uint32_t OwnAddress1 = 0;
  //uint32_t AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  uint32_t DualAddressMode = I2C_DUALADDRESS_DISABLE;
  uint32_t OwnAddress2 = 0;
  uint32_t OwnAddress2Masks = I2C_OA2_NOMASK;
  uint32_t GeneralCallMode = I2C_GENERALCALL_DISABLE;
  uint32_t NoStretchMode = I2C_NOSTRETCH_DISABLE;
  
  
  if(i2cx == I2C2){
      SET_BIT(RCC->APB1ENR1, RCC_APB1ENR1_I2C2EN);
  }
  /*  Disable the selected I2C peripheral 
   *  A software reset can be performed by clearing the PE bit in the I2C_CR1 register.
   *  In that case I2C lines SCL and SDA are released. Internal states machines are reset and
   *  communication control bits, as well as status bits come back to their reset value. The
   *  configuration registers are not impacted.
   */
  CLEAR_BIT(i2cx->CR1, I2C_CR1_PE);
  
  /*---------------------------- I2Cx TIMINGR Configuration ------------------*/
  /* Configure I2Cx: Frequency range */
  i2cx->TIMINGR = Timing & TIMING_CLEAR_MASK;
  
  /*---------------------------- I2Cx OAR1 Configuration ---------------------*/
  /* Disable Own Address1 before set the Own Address1 configuration */
  i2cx->OAR1 &= ~I2C_OAR1_OA1EN;
  
  /* Configure I2Cx: Own Address1 and ack own address1 mode */
  i2cx->OAR1 = (I2C_OAR1_OA1EN | OwnAddress1);
  

  /*---------------------------- I2Cx CR2 Configuration ----------------------*/
  /* Enable the AUTOEND by default, and enable NACK (should be disable only during Slave process */
  i2cx->CR2 |= (I2C_CR2_AUTOEND | I2C_CR2_NACK);
  
  /*---------------------------- I2Cx OAR2 Configuration ---------------------*/
  /* Disable Own Address2 before set the Own Address2 configuration */
  i2cx->OAR2 &= ~I2C_DUALADDRESS_ENABLE;
  
  /* Configure I2Cx: Dual mode and Own Address2 */
  i2cx->OAR2 = (DualAddressMode | OwnAddress2 | (OwnAddress2Masks << 8));

  /*---------------------------- I2Cx CR1 Configuration ----------------------*/
  /* Configure I2Cx: Generalcall and NoStretch mode */
  i2cx->CR1 = (GeneralCallMode | NoStretchMode);

  /* Enable the selected I2C peripheral */
  SET_BIT(i2cx->CR1, I2C_CR1_PE);



}


void  i2c_master_send (I2C_TypeDef *i2cx){
  uint8_t pData[20];      /*!< Pointer to I2C transfer buffer            */
  uint16_t Size = 2;          /*!< I2C transfer counter                      */
  uint32_t tmpreg = 0U;
  
  
  /* Sending Slave Address */
  
  /* Get the CR2 register value */
  tmpreg = i2cx->CR2;
  
  /* clear tmpreg specific bits */
  tmpreg &= (uint32_t)~((uint32_t)(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_STOP));

  /* update tmpreg */
  tmpreg |= (uint32_t)(((uint32_t)DevAddress & I2C_CR2_SADD) | (((uint32_t)Size << 16 ) & I2C_CR2_NBYTES) | \
            (uint32_t)I2C_AUTOEND_MODE | (uint32_t)I2C_GENERATE_START_WRITE);

  /* update CR2 register */
  i2cx->CR2 = tmpreg;
  
  while(Size > 0U)
  {
    /* Wait until TXIS flag is set */
    while(__I2C_GET_FLAG(i2cx, I2C_ISR_TXIS) == RESET)
    {
      flagcheck = (((I2C2->ISR & I2C_ISR_TXIS) == I2C_ISR_TXIS) ? SET : RESET);
      // Acknowledge failed detection during an I2C Communication.
      
    }
      
  
  
  
  
  I2C_WaitOnTXISFlagUntilTimeout
  
  
  
//  while(Size > 0U)
//    {
//      /* Wait until TXIS flag is set */
//      if(I2C_WaitOnTXISFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
//      {
//        if(hi2c->ErrorCode == HAL_I2C_ERROR_AF)
//        {
//          return HAL_ERROR;
//        }
//        else
//        {
//          return HAL_TIMEOUT;
//        }
//      }
//      /* Write data to TXDR */
//      hi2c->Instance->TXDR = (*hi2c->pBuffPtr++);
//      hi2c->XferCount--;
//      hi2c->XferSize--;

//      if((hi2c->XferSize == 0U) && (hi2c->XferCount!=0U))
//      {
//        /* Wait until TCR flag is set */
//        if(I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_TCR, RESET, Timeout, tickstart) != HAL_OK)
//        {
//          return HAL_TIMEOUT;
//        }

//        if(hi2c->XferCount > MAX_NBYTE_SIZE)
//        {
//          hi2c->XferSize = MAX_NBYTE_SIZE;
//          I2C_TransferConfig(hi2c, DevAddress, hi2c->XferSize, I2C_RELOAD_MODE, I2C_NO_STARTSTOP);
//        }
//        else
//        {
//          hi2c->XferSize = hi2c->XferCount;
//          I2C_TransferConfig(hi2c, DevAddress, hi2c->XferSize, I2C_AUTOEND_MODE, I2C_NO_STARTSTOP);
//        }
//      }
//    }

//    /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
//    /* Wait until STOPF flag is set */
//    if(I2C_WaitOnSTOPFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
//    {
//      if(hi2c->ErrorCode == HAL_I2C_ERROR_AF)
//      {
//        return HAL_ERROR;
//      }
//      else
//      {
//        return HAL_TIMEOUT;
//      }
//    }

//    /* Clear STOP Flag */
//    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

//    /* Clear Configuration Register 2 */
//    I2C_RESET_CR2(hi2c);

//    hi2c->State = HAL_I2C_STATE_READY;
//    hi2c->Mode  = HAL_I2C_MODE_NONE;

//    /* Process Unlocked */
//    __HAL_UNLOCK(hi2c);

//    return HAL_OK;
//  }
//  else
//  {
//    return HAL_BUSY;
//  }
  
}


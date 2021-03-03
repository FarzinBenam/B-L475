#ifndef _I2C_H
#define _I2C_H


#include "stm32l4xx.h"                  // Device header
#include "fcmd.h"
#include <stdio.h>


/*******************************************************************************
 * defintions
 *******************************************************************************/


/*******************************************************************************
 * I2C Functions
 *******************************************************************************/
int   I2C_Read      (I2C_TypeDef* I2Cx, uint8_t* buf, uint32_t nbuf, uint8_t SlaveAddress);
int   I2C_Write     (I2C_TypeDef* I2Cx, const uint8_t* buf, uint32_t nbuf,  uint8_t SlaveAddress);
void  I2C_Config    (I2C_TypeDef* I2Cx, int ClockSpeed, int OwnAddress);



#endif
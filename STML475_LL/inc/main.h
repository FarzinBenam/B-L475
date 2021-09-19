/**
  ******************************************************************************
  * @file    main.h 
  * @author  MCD Application Team
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* LL drivers */
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_system.h"
#include "stm32l4xx_ll_utils.h"
//#include "stm32l4xx_ll_pwr.h"
//#include "stm32l4xx_ll_exti.h"
#include "stm32l4xx_ll_gpio.h"
/* LL drivers specific to LL examples IPs */
//#include "stm32l4xx_ll_adc.h"
//#include "stm32l4xx_ll_comp.h"
#include "stm32l4xx_ll_cortex.h"
#include "stm32l4xx_ll_crc.h"
//#include "stm32l4xx_ll_dac.h"
//#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_i2c.h"
//#include "stm32l4xx_ll_iwdg.h"
//#include "stm32l4xx_ll_lptim.h"
//#include "stm32l4xx_ll_lpuart.h"
//#include "stm32l4xx_ll_opamp.h"
//#include "stm32l4xx_ll_rng.h"
#include "stm32l4xx_ll_rtc.h"
#include "stm32l4xx_ll_spi.h"
//#include "stm32l4xx_ll_swpmi.h"
//#include "stm32l4xx_ll_tim.h"
#include "stm32l4xx_ll_usart.h"
//#include "stm32l4xx_ll_wwdg.h"

/* FCMD Libs */
#include "configs.h"
#include "uart.h"
#include "qspi.h"
#include "fs.h"
#include "../../Libraries/STM32L4_v1.17.0/Drivers/BSP/Components/mx25r6435f/mx25r6435f.h"


#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Exported types ------------------------------------------------------------*/
/* define a structure for RTC_TR register with bit fields */
typedef struct{
	uint32_t second		:7;
	uint32_t reserved1:1;
	uint32_t minute		:7;
	uint32_t reserved2:1;
	uint32_t hour			:6;
	uint32_t ampm			:1;
	uint32_t reserved3:9;
}rtc_time_params;

/* define a structure for RTC_DR register with bit fields */
typedef struct{
	uint32_t Day			:6;
	uint32_t reserved1:2;
	uint32_t month		:5;
	uint32_t weekday	:3;
	uint32_t year			:8;
	uint32_t reserved2:8;
}rtc_date_params;


/* ==============   BOARD SPECIFIC CONFIGURATION CODE BEGIN    ============== */
/**
  * @brief GPIO Clock Enable Macros
  */
#define GPIOA_CLK_ENABLE()                 LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
#define GPIOB_CLK_ENABLE()                 LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
#define GPIOC_CLK_ENABLE()                 LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
#define GPIOD_CLK_ENABLE()                 LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOD);
#define GPIOE_CLK_ENABLE()                 LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOE);
/**
  * @brief USART Clock Enable Macros
  */
#define USART1_CLK_ENABLE()               LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
#define USART2_CLK_ENABLE()               LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
#define USART3_CLK_ENABLE()               LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
#define USART4_CLK_ENABLE()               LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART4);
/**
  * @brief I2C Clock Enable Macros
  */
#define I2C1_CLK_ENABLE()                 LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
#define I2C2_CLK_ENABLE()                 LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C2);
#define I2C3_CLK_ENABLE()                 LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C3);

/**
  * @brief LED1
  */
#define LED1_PIN                           LL_GPIO_PIN_5
#define LED1_GPIO_PORT                     GPIOA
#define LED1_GPIO_CLK_ENABLE()             LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA)
#define LED1_ON()		                       SET_BIT(LED1_GPIO_PORT->BSRR, LED1_PIN)
#define LED1_OFF()	                       SET_BIT(LED1_GPIO_PORT->BRR, LED1_PIN)

/**
  * @brief LED2
  */
#define LED2_PIN                           LL_GPIO_PIN_14
#define LED2_GPIO_PORT                     GPIOB
#define LED2_GPIO_CLK_ENABLE()             LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB)
#define LED2_ON()                          SET_BIT(LED2_GPIO_PORT->BSRR, LED2_PIN)
#define LED2_OFF()	                       SET_BIT(LED2_GPIO_PORT->BRR, LED2_PIN)

/**
  * @brief LED3
  */
#define LED3_PIN                           LL_GPIO_PIN_9
#define LED3_GPIO_PORT                     GPIOC
#define LED3_GPIO_CLK_ENABLE()             LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC)
#define LED3_ON()		                       SET_BIT(LED3_GPIO_PORT->BSRR, LED3_PIN)
#define LED3_OFF()	                       SET_BIT(LED3_GPIO_PORT->BRR, LED3_PIN)


/**
  * @brief Joystick Sel push-button
  */
#define USER_BUTTON_PIN                         LL_GPIO_PIN_13
#define USER_BUTTON_GPIO_PORT                   GPIOC
#define USER_BUTTON_GPIO_CLK_ENABLE()           LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC)
#define USER_BUTTON_EXTI_LINE                   LL_EXTI_LINE_13
#define USER_BUTTON_EXTI_IRQn                   EXTI15_10_IRQn
#define USER_BUTTON_EXTI_LINE_ENABLE()          LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_13)   
#define USER_BUTTON_EXTI_FALLING_TRIG_ENABLE()  LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_13)   
#define USER_BUTTON_SYSCFG_SET_EXTI()           do {                                                                     \
                                                  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);                  \
                                                  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_EXTI_LINE_13);  \
                                                } while(0)
#define USER_BUTTON_IRQHANDLER                  EXTI15_10_IRQHandler

/**
  * @brief HTS221 defines
  */

                                                
/* ==============   BOARD SPECIFIC CONFIGURATION CODE END      ============== */

/**
  * @brief Toggle periods for various blinking modes
  */
#define LED_BLINK_FAST  200
#define LED_BLINK_SLOW  500
#define LED_BLINK_ERROR 1000

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
                                                
                                                
#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

#include "gpio.h"

/* Private defines -----------------------------------------------------------*/
/** @defgroup GPIO_Private_Defines GPIO Private Defines
  * @{
  */
#define GPIO_MODE             ((uint32_t)0x00000003)
#define ANALOG_MODE           ((uint32_t)0x00000008)
#define EXTI_MODE             ((uint32_t)0x10000000)
#define GPIO_MODE_IT          ((uint32_t)0x00010000)
#define GPIO_MODE_EVT         ((uint32_t)0x00020000)
#define RISING_EDGE           ((uint32_t)0x00100000)
#define FALLING_EDGE          ((uint32_t)0x00200000)
#define GPIO_OUTPUT_TYPE      ((uint32_t)0x00000010)

#define GPIO_NUMBER           ((uint32_t)16)

/** @defgroup GPIO_Exported_Functions_Group1 Initialization/de-initialization functions 
 *  @brief    Initialization and Configuration functions
 *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================

@endverbatim
  * @{
  */
  
/**
  * @brief  Initialize the GPIOx peripheral according to the specified parameters in the GPIO_Init.
  * @param  GPIOx: where x can be (A..H) to select the GPIO peripheral for STM32L4 family
  * @param  GPIO_Init: pointer to a GPIO_InitTypeDef structure that contains
  *         the configuration information for the specified GPIO peripheral.
  * @retval None
  */
void GPIO_Init (GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init)
{
  uint32_t position = 0x00;
  uint32_t iocurrent = 0x00;    // current pin
  uint32_t temp = 0x00;

  
  
  // enabling the related GPIO Clock
  if(GPIOx == GPIOA){
    SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIOAEN);	
  }
  else if(GPIOx == GPIOB){
    SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIOBEN);
  }
  else if(GPIOx == GPIOC){
    SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIOCEN);
   }
  else if(GPIOx == GPIOD){
    SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIODEN);
  }
  else if(GPIOx == GPIOE){
    SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIOEEN);
  }
  

  /* Configure the port pins */
  while (((GPIO_Init->Pin) >> position) != RESET) // looking for the pin position to configure
  {
    /* Get current io position */
    iocurrent = (GPIO_Init->Pin) & (1U << position);
    

    if(iocurrent)
    {
      /*--------------------- GPIO Mode Configuration ------------------------*/
      /* In case of Alternate function mode selection */
      if((GPIO_Init->Mode == GPIO_MODE_AF_PP) || (GPIO_Init->Mode == GPIO_MODE_AF_OD))
      {
        /* Configure Alternate function mapped with the current IO */
        temp = GPIOx->AFR[position >> 3];
        temp &= ~((uint32_t)0xF << ((uint32_t)(position & (uint32_t)0x07) * 4)) ;
        temp |= ((uint32_t)(GPIO_Init->Alternate) << (((uint32_t)position & (uint32_t)0x07) * 4));
        GPIOx->AFR[position >> 3] = temp;
      }

      /* Configure IO Direction mode (Input, Output, Alternate or Analog) */
      temp = GPIOx->MODER;
      temp &= ~(GPIO_MODER_MODE0 << (position * 2));
      temp |= ((GPIO_Init->Mode & GPIO_MODE) << (position * 2));
      GPIOx->MODER = temp;

      
      
      

      /* In case of Output or Alternate function mode selection */
      if((GPIO_Init->Mode == GPIO_MODE_OUTPUT_PP) || (GPIO_Init->Mode == GPIO_MODE_AF_PP) ||
         (GPIO_Init->Mode == GPIO_MODE_OUTPUT_OD) || (GPIO_Init->Mode == GPIO_MODE_AF_OD))
      {
        /* Configure the IO Speed */
        temp = GPIOx->OSPEEDR;
        temp &= ~(GPIO_OSPEEDR_OSPEED0 << (position * 2));
        temp |= (GPIO_Init->Speed << (position * 2));
        GPIOx->OSPEEDR = temp;

        /* Configure the IO Output Type */
        temp = GPIOx->OTYPER;
        temp &= ~(GPIO_OTYPER_OT0 << position) ;
        temp |= (((GPIO_Init->Mode & GPIO_OUTPUT_TYPE) >> 4) << position);
        GPIOx->OTYPER = temp;
      }

#if defined(STM32L471xx) || defined(STM32L475xx) || defined(STM32L476xx) || defined(STM32L485xx) || defined(STM32L486xx)

      /* In case of Analog mode, check if ADC control mode is selected */
      if((GPIO_Init->Mode & GPIO_MODE_ANALOG) == GPIO_MODE_ANALOG)
      {
        /* Configure the IO Output Type */
        temp = GPIOx->ASCR;
        temp &= ~(GPIO_ASCR_ASC0 << position) ;
        temp |= (((GPIO_Init->Mode & ANALOG_MODE) >> 3) << position);
        GPIOx->ASCR = temp;
      }

#endif /* STM32L471xx || STM32L475xx || STM32L476xx || STM32L485xx || STM32L486xx */

      /* Activate the Pull-up or Pull down resistor for the current IO */
      temp = GPIOx->PUPDR;
      temp &= ~(GPIO_PUPDR_PUPD0 << (position * 2));
      temp |= ((GPIO_Init->Pull) << (position * 2));
      GPIOx->PUPDR = temp;

//      /*--------------------- EXTI Mode Configuration ------------------------*/
//      /* Configure the External Interrupt or event for the current IO */
//      if((GPIO_Init->Mode & EXTI_MODE) == EXTI_MODE)
//      {
//        /* Enable SYSCFG Clock */
//        __HAL_RCC_SYSCFG_CLK_ENABLE();

//        temp = SYSCFG->EXTICR[position >> 2];
//        temp &= ~(((uint32_t)0x0F) << (4 * (position & 0x03)));
//        temp |= (GPIO_GET_INDEX(GPIOx) << (4 * (position & 0x03)));
//        SYSCFG->EXTICR[position >> 2] = temp;

//        /* Clear EXTI line configuration */
//        temp = EXTI->IMR1;
//        temp &= ~((uint32_t)iocurrent);
//        if((GPIO_Init->Mode & GPIO_MODE_IT) == GPIO_MODE_IT)
//        {
//          temp |= iocurrent;
//        }
//        EXTI->IMR1 = temp;

//        temp = EXTI->EMR1;
//        temp &= ~((uint32_t)iocurrent);
//        if((GPIO_Init->Mode & GPIO_MODE_EVT) == GPIO_MODE_EVT)
//        {
//          temp |= iocurrent;
//        }
//        EXTI->EMR1 = temp;

//        /* Clear Rising Falling edge configuration */
//        temp = EXTI->RTSR1;
//        temp &= ~((uint32_t)iocurrent);
//        if((GPIO_Init->Mode & RISING_EDGE) == RISING_EDGE)
//        {
//          temp |= iocurrent;
//        }
//        EXTI->RTSR1 = temp;

//        temp = EXTI->FTSR1;
//        temp &= ~((uint32_t)iocurrent);
//        if((GPIO_Init->Mode & FALLING_EDGE) == FALLING_EDGE)
//        {
//          temp |= iocurrent;
//        }
//        EXTI->FTSR1 = temp;
//      }
    }
    
    position++;
  }
}



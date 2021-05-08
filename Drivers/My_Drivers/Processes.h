#include "main.h"



#define GPIO_NUMBER           (16u)

void	OsInits (void);

/*******************************************************************************/ 
// FCMD Processes
void	time				(uint8_t *cmdbuffer);
void	Set_time			(uint8_t *cmdbuffer);
void	WiFi				(uint8_t *cmdbuffer);
void	Wifilisten          (uint8_t *cmdbuffer);
void	Help                (uint8_t *cmdbuffer);
/*******************************************************************************/ 
// Configurations
void	RCC_Config 			(void);

void	ISR_Config			(void);
void	RTC_Config			(void);
void	Systick_EN			(int tick);
uint32_t getTick			(void);
void	wifi_init 			(void);

// Pheripheral Funtions
void	WIFI_wakeup			(void);
int		wifi_recieve		(void);
void	wifi_cmd			(uint8_t *pdata);
void	wifi_init_cmds		(void);
void	wifi_Connect		(void);
int		wifi_cmd_send		(uint8_t *pdata);
void	_wifi_send			(const char *format, ...);
int		_wifi_read			(void);

void	time_UTC			(int epoch);

void	OsInits				(void);



// The FreeRTOS task functions prototype
void    send1Task           (void *argument);
void    send2Task           (void *argument);
void    CmderStatusPoll     (void *argument);
void    led1Task            (void *argument);
void    led2Task            (void *argument);
void    TimeTerminal        (void *argument);


void	delayS				(int Seconds );
void	delayMs				(int MilliSecond);
void	halt				(int num);
void	Welcome				(void);
void	line				(int leng);
void	BckSpc				(uint8_t Line);
void	nl					(int Count);
void	leds				(void);
void	led1				(void);
void	led2				(void);
void    togglepin 		    (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);



// Interrupts
void	EXTI15_10_IRQHandler	(void);
void	USART1_IRQHandler		(void);


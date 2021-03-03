#include "fcmd.h"


void	OsInits (void);

/*******************************************************************************/ 
// Configurations
void	RCC_Config 				(void);
void	GPIO_Config				(void);
void	ISR_Config				(void);
void	RTC_Config				(void);
void	Systick_EN				(int tick);
uint32_t getTick				(void);
void	wifi_init 			(void);

// Pheripheral Funtions
void	WIFI_wakeup					(void);
int		wifi_recieve			(void);
void	wifi_cmd					(uint8_t *pdata);
void	wifi_init_cmds		(void);
void	wifi_Connect			(void);
int		wifi_cmd_send			(uint8_t *pdata);
void	_wifi_send				(const char *format, ...);
int		_wifi_read				(void);



void	OsInits						(void);
uint8_t USART_Process		(void);
void	cmd_process_inits	(void);
uint8_t		add_command		(const char *cmd, void (*functionptr)(void));
void	cmd_entry					(void);
void	cmd_exit					(void);
uint8_t		cmd_decode		(uint8_t *cmd, const char *OsCmds);
void	time							(void);
void	Set_time					(void);
void	time_UTC					(int epoch);
void	WiFi							(void);





void	delayS		(int Seconds );
void	delayMs		(int MilliSecond);
void	halt			(int num);
void	Welcome		(void);
void	line			(int leng);
void	BckSpc		(uint8_t Line);
void	nl				(int Count);
void	leds			(void);
void	led1			(void);
void	led2			(void);




// Interrupts
void	EXTI15_10_IRQHandler	(void);
void	USART1_IRQHandler			(void);


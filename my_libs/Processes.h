#include "fcmd.h"


void	OsInits (void);

/*******************************************************************************/ 
// Configurations
void	RCC_Config 				(void);
void	GPIO_Config				(void);
void	ISR_Config				(void);
void	USART1_Config			(void);
void	USART3_Config 		(void);
void	RTC_Config				(void);
void	SPI3_Config 			(void);
void	Systick_EN				(int tick);
uint32_t getTick				(void);
void	wifi_init 			(void);

// Pheripheral Funtions
void	WIFI_Init					(void);
int		SPI_Transmit			(uint8_t *pData, uint16_t Size);
int		spi_wifi_send			(uint8_t *pdata,  uint16_t len);
int		spi_recieve8			(uint8_t dat);
int		SPI_Receive16			(uint8_t *pData, int cmd);
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


void	_usart1_send_b						(int ch);
void	_usart1_send_s						(const char Message[]);
void	_usart1_printf							(const char *format, ...);
int		read							(void);
void	_usart3_send_b	(int ch);
void	_usart3_send_s		(const char Message[]);



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


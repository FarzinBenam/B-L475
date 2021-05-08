#ifndef FCMD_H_
#define FCMD_H_

#include "main.h"

//************************************************************************
//*	CMD declarations and related parameters on the PROM                  
//************************************************************************
// Kernel Specific Constants
static const char _CMD_Time[]		= "TIME";
static const char _CMD_tmp[]		= "TMP";
static const char _CMD_lcd[]		= "LCD";
static const char _CMD_tmpbck[]		= "TMPBCK";
static const char _CMD_tmprep[]		= "TMPREP";
static const char _CMD_tmpbckoff[]	= "TMPBCKOFF";
static const char _CMD_tmpbckread[]	= "TMPBCKREAD";
static const char _CMD_date[]		= "DATE";
static const char _CMD_alarm[]		= "ALARM";
static const char _CMD_wifi[]		= "WIFI";
static const char _CMD_wifilisten[] = "WIFILISTEN";
static const char _CMD_help[]		= "HELP";

static const char _Param_set[]		= "SET";
//******************************************
// Kernel Specific Constants
static const char wc_note[]			= "WELCOME TO FCMD";
static const char CmdError_1[]		= "[!] unknown Command!"; //Enter HELP for more information.";
static const char CmdError_2[]		= "[!] Wrong Input!";
static const char UpatedNote[]		= "updated!";
static const char New_Line[]		= "\n\r";

// Temperature Specific Constants
static const char TMPNote[]			= "Temperature";
static unsigned char month_days[12]	= {31,28,31,30,31,30,31,31,30,31,30,31};
static unsigned char week_days[7]	= {4,5,6,0,1,2,3};






//************************************************************************
//* Specific Structure Definitions
//************************************************************************
typedef struct{
	uint8_t	ProgramStatus :1;
	//CmdStatus = 0 -> normal status
	//CmdStatus = 1 -> command Entered
	//CmdStatus = 2 -> cmd without parameter
	//CmdStatus = 3 -> cmd with parameter
	//CmdStatus = 4 -> Usart RX intterupt
	//
	//
	//
	uint8_t	CmdStatus :3;
	uint8_t	CmdProcessStatus :1; 
	uint8_t	System1SecStatus1 :1;
	uint8_t	System1SecStatus2 :1;
	uint8_t	Unused:1;
}Status;
typedef struct{
    // Pointer to a CMD
	const char *cmd_name;
	/* function pointer Declaration:
    // return_type (*function_name)(arguments)    
    */
	void (*cmd_func_ptr)(uint8_t buffer[]);  
} Commands;

//************************************************************************
//* FCMD Function Definitions
//************************************************************************
uint8_t USART_Process		(void);
void	cmd_process_inits	(void);
uint8_t add_command		    (const char *cmd, void (*function_ptr)(uint8_t buffer[]));
void	cmd_entry			(void);
void	cmd_exit			(void);
uint8_t	cmd_decode		    (uint8_t *cmd, const char *OsCmds);

#endif  /* FCMD_H_ */
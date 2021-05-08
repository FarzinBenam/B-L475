#include "fcmd.h"

extern volatile uint8_t	CmdStatus;
extern volatile uint8_t	WifiStatus;
extern volatile uint8_t	wificmdStatus;

// Length of the inpur CMD
static uint8_t	CmdLenght;
// Buffer for the inpur CMD
static uint8_t	CmdBuffer[CMD_BUFFER_SIZE];
static uint8_t  LastSuccessfullcmd[CMD_BUFFER_SIZE];

// Declare SystemStatus of type Status
static Status   SysStatus;
static Commands	Cmds[MAX_COMMANDS]; // array of static structures 

//************************************************************************
//* FCMD Kernel Processes
//************************************************************************
uint8_t USART_Process	(void)
{	
	uint8_t tmp = USART1->RDR;
	
    if (CmdLenght){
        if(tmp == BACKSPACE1 || tmp == BACKSPACE2){ // backspace session
            _usart1_send_b(tmp);
            --CmdLenght;
            goto Return;
        }
    } else if(tmp == SCAPE_BUT){                    // lastcmd session
        terminal("%s", LastSuccessfullcmd);
        CmdLenght = sizeof(LastSuccessfullcmd);
        memcpy(CmdBuffer, LastSuccessfullcmd,sizeof(LastSuccessfullcmd));
        goto Return;
    }
    
    
    
	
	if (tmp == ENTER){
			// Terminating with Enter and Null 
	    CmdBuffer[CmdLenght] = 0x00;
	    nl(1);
			//CmdStatus = 2 -> command Entered
	    CmdStatus = 2;
	    CmdLenght = 0;
        
        
	    return CmdStatus;
	}
	
	if (CmdLenght >= CMD_BUFFER_SIZE) goto Return;
	
	CmdBuffer[CmdLenght] = tmp;
	// printing the inserted value
	_usart1_send_b(CmdBuffer[CmdLenght++]);
	
	// Normal Return
	Return:

	//CmdStatus = 0 -> normal status
	CmdStatus = 0;
	
	return CmdStatus;
}


void	cmd_process_inits	(void)
{
	Cmds[0].cmd_name = 0;	// Initializing commands structure.

	// Adding new Commands to system
	add_command(_CMD_Time, &time);
	add_command(_CMD_wifi, &WiFi);
    add_command(_CMD_wifilisten, &Wifilisten);
	add_command(_CMD_help, &Help);
  //add_command(_CMD_tmp, &temprat);
	//add_command(_CMD_lcd, &lcd);
	//add_command(_CMD_date, &date);
}

uint8_t add_command		(const char *cmd, void (*function_ptr)(uint8_t buffer[]))
{
    uint8_t idx;	// index
	
    for (idx = 0; idx <= MAX_COMMANDS-1; ++idx)
	{
	    // search for an empty slot
		if (Cmds[idx].cmd_name == 0)
		{
		    Cmds[idx].cmd_name      = cmd;          // adding the new command to the index of cmds
		    Cmds[idx].cmd_func_ptr  = function_ptr; // adding the function pointer of related cmd to indexx
		    ++idx;
		    Cmds[idx].cmd_name      = 0;
		    return idx;
	    }
    }
    CmdLenght = 0;
    return 0;
}

void	cmd_entry		(void)
{
	uint8_t idx;
	uint8_t temp;	
	
    
    
	cmd_process_inits();
	
	for (idx = 0 ; idx <= MAX_COMMANDS-1; ++idx){
		// break if buffer reach to empty slot
		if (Cmds[idx].cmd_name == 0x00) break;
		
		// Sending user cmd and org cmd to compare by cmd_decode
		temp = cmd_decode(CmdBuffer, Cmds[idx].cmd_name);
		
		
		// execute the process
		if (temp){
            // executing the wanted command
			(*Cmds[idx].cmd_func_ptr)(CmdBuffer);
            // saving the last command
            memcpy(LastSuccessfullcmd, CmdBuffer, sizeof(CmdBuffer));
			return;
		}
	}
    terminal("%s ?!", CmdBuffer);
    nl(1);
	write(CmdError_1);
	nl(1);
	cmd_exit();
}
void	cmd_exit		(void)
{
	CmdStatus = 0;
	CmdLenght = 0;
	WifiStatus = 1;
	nl(1);
	_usart1_send_b(CommandSign);
}

uint8_t cmd_decode		(uint8_t *cmd, const char *OsCmds)    
{
	volatile uint8_t idxUserCmd = 0;
	uint8_t temp;
	
	
	while(1){
		temp = cmd[idxUserCmd];
		

		
		// Both uppercase or lowercase will be tested
		if ((*OsCmds == temp)||(*OsCmds == (temp - 0x20)))
		{
			/*	Both reached to zero bytes means command is matched
				and cmd is without parameter */
			if ((*OsCmds == 0x00) && (cmd[idxUserCmd] == 0x00))
			{
				//CmdStatus = 2 -> cmd without parameter
				SysStatus.CmdStatus = 2;
				return 1;
			}
			/*	Org cmd reached to zero bytes and other to SPACE means command is matched
				and cmd is with parameter */
			if ((*OsCmds == 0x00) && (cmd[idxUserCmd] == SPACE))
			{
				//CmdStatus = 3 -> cmd with parameter
				SysStatus.CmdStatus = 3;
				return 1;
			}
			++idxUserCmd;
			++OsCmds;
		}
		else return 0; // wrong cmd
	}
}


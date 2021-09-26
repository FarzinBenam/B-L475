/**
  
  * @file			FS
  * @author		Farzin M.Benam
  * @version	
  * @date     2021-08-21
  * @brief		log file system
  *             
  *******************************************************************************/ 
#include "fs.h"

/* Exported constants --------------------------------------------------------*/
extern const char	_TEMPERATURE_HUMIDITY_FOLDER_NAME[];
extern const char	start_tag[];
extern const char	end_tag[];
/* Private define ------------------------------------------------------------*/
#define		T_H_FOLDER_START_ADDR	QSPI_START_ADDRESS
#define		T_H_FOLDER_END_ADDR		0x12000
#define		T_H_LOG_LENGTH				23

static int _fs_search_free_sector (fs_folder_def *fsfolderint, int size);
static int _fs_search_last_log		(fs_folder_def *fsfolder);

union T_H_LOG_Parameters{
	uint32_t	sensor_32b;
	uint8_t		sensor_8b;
};
union char_to_16hex{
	uint16_t	num16bit;
	uint8_t		nim8bit[2];
};



uint8_t QSPI_Buffer[MX25R6435F_SECTOR_SIZE];

/*******************************************************************************
 * 
 *******************************************************************************/

static int _fs_search_last_log			(fs_folder_def *fsfolder)
{
	volatile uint16_t i;
	uint8_t k;
	int address = fsfolder->StartAddr;
	volatile uint32_t count = 0;
	char tags[4];
	
	// gather start and end tag in an array to search for them
	sprintf(tags, "%s%s", start_tag, end_tag);
	
	fsfolder->LastLogAddr = fsfolder->StartAddr;
	k = 0;
	
	// finding the first log
	terminal("\nLog Search started from address:    0x%X", fsfolder->LastLogAddr);
	while(address < fsfolder->EndAddr){
		qspi_ReadMemory(QSPI_Buffer, address, MX25R6435F_SECTOR_SIZE);
		i = 0; 

		// searching for the tags (start and end)
		while(i < MX25R6435F_SECTOR_SIZE){
			// searching for the start and end tags
			if(QSPI_Buffer[i++] == tags[k]){
				k++;
				
				if(k == 4)		// if both tags are found save the last logs address	
				{
					fsfolder->LastLogAddr = (address + i);
					count++;
					k = 0;
				}
				
			}
		}

		address += MX25R6435F_SECTOR_SIZE;
	}
	
	
						
					
					
	// a log found
	terminal("\n%d Log found, last address is at : 0x%X", count, fsfolder->LastLogAddr);
	return SUCCESS;

////						temp[0] = QSPI_Buffer[++i];
////						temp[1] = QSPI_Buffer[++i];
////						temp[2] = QSPI_Buffer[++i];
////						temp[3] = QSPI_Buffer[++i];
////						temp[4] = NULL;
////						sscanf(temp, "%X", (uint8_t *)temp1);
////						
////						data_length.nim8bit[0] = temp1[0];
////						data_length.nim8bit[1] = temp1[1];
		
	Error:
	return ERROR;
}


/* Temperature and Humidity log ----------------------------------------------*/
void	T_H_log_init (fs_folder_def *fsfolder)
{
//	strcpy(fsfolder->Name, _TEMPERATURE_HUMIDITY_FOLDER_NAME); // be carefull with the size of array
//  strcpy(fsfolder->Commant, "the folder used for stroing the temperature and Humidity logs");
	fsfolder->StartAddr = T_H_FOLDER_START_ADDR;
	fsfolder->EndAddr = 	T_H_FOLDER_END_ADDR;
	
	_fs_search_last_log(fsfolder);
	terminal("\nlast log address: 0x%X", fsfolder->LastLogAddr);
}
void	T_H_log		(fs_folder_def *fsfolder)
{
	LL_RTC_TimeTypeDef time;
	LL_RTC_DateTypeDef date;
	union T_H_LOG_Parameters sensor;
	uint8_t	temp[3];
	uint16_t str_length;
	
	sensor.sensor_32b = HTS221_T_ReadTemp(HTS221_SLAVE_ADD);
	temp[0] = sensor.sensor_8b;

	sensor.sensor_32b = HTS221_H_ReadHumidity(HTS221_SLAVE_ADD);
	temp[1] = sensor.sensor_8b;
	
	time_read(&time, &date);
//	terminal("\n%.2X:%.2X:%.2X ",time.Hours, time.Minutes, time.Seconds);
//	terminal("\n20%.2X-%.2X-%.2X", date.Year, date.Month, date.Day);
//	terminal("\nT: %.2d H: %.2d", temp[0], temp[1]);
	sprintf(	(char*)QSPI_Buffer, "%.2X%.2X%.2X%.2X%.2X%.2d%.2d",
						time.Hours,
						time.Minutes,
						date.Year,
						date.Month,
						date.Day,
						temp[0],
						temp[1]);

	str_length = strlen((char*)QSPI_Buffer);
//	terminal("\n%s    length:%d\n", buffer, str_length);
//	terminal("\nCRC length: %d",str_length);
	temp[2] = crc_8bit((uint8_t*)QSPI_Buffer, str_length);
	
	
//	sprintf(	(char*)QSPI_Buffer, "%.2s%.4X%.2X%.2X%.2X%.2X%.2X%.2d%.2d%.2X%.2s",
//						start_tag,
//						str_length,	// data length
//						time.Hours,
//						time.Minutes,
//						date.Year,
//						date.Month,
//						date.Day,
//						temp[0],	// Temperature
//						temp[1],	// Humidity
//						temp[2],	// CRC
//						end_tag);
//	str_length = strlen((char*)QSPI_Buffer);
//	fsfolder->LogLength = str_length;
//	terminal("\n%s    length:%d\n", buffer, str_length);
//	for(int i = 0; i < str_length; i++){		// printing the read buffer
//			terminal("%.2X ", buffer[i]);
//	}
	
	_fs_log((char*)QSPI_Buffer, fsfolder);
}
void	_fs_log (char *data, fs_folder_def *fsfolder)
{
	uint8_t i;
	int address;

// 	terminal("\nBuffer:\n");
//	for(i = 0; data[i]; i++){
//		terminal("%X-", data[i]);
//	}
//	terminal("\nLength: %d",strlen((char*)QSPI_Buffer));
//	nl(2); 
//	while(1);
	
		// check for the end of spi flash memory
	if (fsfolder->LastLogAddr >= fsfolder->EndAddr){
		terminal("\nEnd of flash memory!\nMemory is FULL!\n");
		return;
	}
	// adding header to the log
	
	qspi_Byte_write_init(fsfolder->LastLogAddr);
	//qspi_WriteMemory((uint8_t*)QSPI_Buffer, fsfolder->LastLogAddr, fsfolder->LogLength);
	fsfolder->LastLogAddr += fsfolder->LogLength;
//	terminal("\nlast log address: 0x%X ", fsfolder->LastLogAddr);
	
}

/******************* (C) COPYRIGHT 2021 Farzin_M.Benam *****END OF FILE****/

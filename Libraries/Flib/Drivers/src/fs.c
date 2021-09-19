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
#define		T_H_FOLDER_START_ADDR	00//QSPI_START_ADDRESS
#define		T_H_FOLDER_END_ADDR		0x800000
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

void	_fs_log (char *data, fs_folder_def *fsfolder)
{
	uint8_t i;
	int address;

// 	terminal("\nBuffer:\n");
//	for(i = 0; i < 25; i++){
//		terminal("%X-", data[i]);
//	}
//	nl(2); 
	
	address = _fs_search_last_log(fsfolder);
	terminal("\naddr:0x%X, length: %d", address, strlen(data));
	while(1);
	qspi_WriteMemory((uint8_t*)data, address, strlen(data));
  
	
}


ErrorStatus	_fs_new_folder (fs_folder_def *fsfolder)
{
//	int	sector_size = (fsfolder->FolderSize / MX25R6435F_SECTOR_SIZE + 1);
//	char buffer[sizeof(fsfolder->Name)];	
//	uint32_t temp;		// used for 32 bit address of the spi flash
//	uint8_t temp1;		// address increment of the buffer */
//	char buff[50];
//	
//	//terminal("\nNew Folder: %d Sectors", sector_size);
//	
//   if(_fs_search_free_sector(fsfolder, sector_size) == ERROR){
//    return ERROR;
//  }
//	
//	
  return SUCCESS;
}

/**
* @brief  Searching for erased sectors to define a folder
* @param  size : size of the desired folder as sectors (4KB)
* @retval 
*/
static int	_fs_search_free_sector	(fs_folder_def *fsfolder, int size)
{
//  volatile uint8_t i, j = 0;
//	uint8_t temp[20];
//  int address = QSPI_START_ADDRESS;
//  int sectorsize = size;
//  /*----------------------------------------------------------------------------*/
//  while(sectorsize){
//    qspi_ReadMemory(temp, address, 3);
//    
//    for(i = 0; i < 3; i++){
//      if(temp[i] != 0xFF){
//        address += MX25R6435F_SECTOR_SIZE;
//        i = 5;			// use it later to reset the search index
//				break;
//      }
//      if(address == 0x7FF000){
//        terminal("\nError!\nEnd of SPI Flash, no free sector found!");
//        return ERROR;
//      }
//    }
//    
//		if(i == 3){
//      sectorsize--;
//			fsfolder->Addr[j++] = address;
//      address += MX25R6435F_SECTOR_SIZE;
//    }
//    if(i == 5){
//      sectorsize = size;
//			j = 0;			// to find the free sectors without any gap (consecutively) 
//    }
//  }
  return SUCCESS;
}
static int _fs_search_last_log			(fs_folder_def *fsfolder)
{
	volatile uint16_t i;
	uint8_t k;
	int address = fsfolder->StartAddr;
	volatile uint32_t count = 0;
	char *tag;
	
	
	fsfolder->LastLogAddr = fsfolder->StartAddr;
	// pointing to the start tag to start searching for the it
	tag = &start_tag;
	k = 0;
	
	// finding the first log
	terminal("\nLog Search started from address:    0x%X", fsfolder->LastLogAddr);
	while(address < fsfolder->EndAddr){
		qspi_ReadMemory(QSPI_Buffer, address, MX25R6435F_SECTOR_SIZE);
		
		i = 0; 

		// searching for the tags (start and end)
		while(i < MX25R6435F_SECTOR_SIZE){
			// searching for the start tag
			if(QSPI_Buffer[i++] == tag[k]){
				k++;
				if(k == 2){
					k = 0;
					if(tag == &start_tag){				// finding the start tag
						tag = &end_tag;
					}
					else 
					{					// finding the end tag and increment the last log address
						fsfolder->LastLogAddr = (address + i);
						count++;
						tag = &start_tag;
					}
				}

			}
			else
			{
				
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
	strcpy(fsfolder->Name, _TEMPERATURE_HUMIDITY_FOLDER_NAME); // be carefull with the size of array
  strcpy(fsfolder->Commant, "the folder used for stroing the temperature and Humidity logs");
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
	char		buffer[50];
	uint8_t	temp[3];
	uint16_t str_length;
	
	
	// check for the end of spi flash memory
	if (fsfolder->LastLogAddr >= fsfolder->EndAddr){
		terminal("\nEnd of flash memory!");
		while(1);
		return;
	}		
	
	sensor.sensor_32b = HTS221_T_ReadTemp(HTS221_SLAVE_ADD);
	temp[0] = sensor.sensor_8b;

	sensor.sensor_32b = HTS221_H_ReadHumidity(HTS221_SLAVE_ADD);
	temp[1] = sensor.sensor_8b;
	
	time_read(&time, &date);
//	terminal("\n%.2X:%.2X:%.2X ",time.Hours, time.Minutes, time.Seconds);
//	terminal("\n20%.2X-%.2X-%.2X", date.Year, date.Month, date.Day);
//	terminal("\nT: %.2d H: %.2d", temp[0], temp[1]);
	sprintf(	buffer, "%.2X%.2X%.2X%.2X%.2X%.2d%.2d",
						time.Hours,
						time.Minutes,
						date.Year,
						date.Month,
						date.Day,
						temp[0],
						temp[1]);

	str_length = strlen(buffer);
//	terminal("\n%s    length:%d\n", buffer, str_length);
//	terminal("\nCRC length: %d",str_length);
	temp[2] = crc_8bit((uint8_t*)buffer, str_length);
	
	
	sprintf(	buffer, "%.2s%.4X%.2X%.2X%.2X%.2X%.2X%.2d%.2d%.2X%.2s",
						start_tag,
						str_length,	// data length
						time.Hours,
						time.Minutes,
						date.Year,
						date.Month,
						date.Day,
						temp[0],	// Temperature
						temp[1],	// Humidity
						temp[2],	// CRC
						end_tag);
	str_length = strlen(buffer);					
//	terminal("\n%s    length:%d\n", buffer, str_length);
//	for(int i = 0; i < str_length; i++){		// printing the read buffer
//			terminal("%.2X ", buffer[i]);
//	}
	
	qspi_WriteMemory((uint8_t*)buffer, fsfolder->LastLogAddr, str_length);
	fsfolder->LastLogAddr += str_length;
//	terminal("\nlast log address: 0x%X ", fsfolder->LastLogAddr);
}



/******************* (C) COPYRIGHT 2021 Farzin_M.Benam *****END OF FILE****/

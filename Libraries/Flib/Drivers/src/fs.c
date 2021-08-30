/**
  
  * @file			
  * @author		Farzin M.Benam
  * @version	
  * @date     2021-08-21
  * @brief		
  *             
  *******************************************************************************/ 
#include "fs.h"


static int _fs_search_free_sector (int size);
/*******************************************************************************
 * 
 *******************************************************************************/

void  _fs_log (_fs_init_def fs)
{
  terminal("\nLOG:\nT:%.2d_H:%.2d'- .'", fs.buffer[0], fs.buffer[1]);
  
  
  
}


ErrorStatus  _fs_new_folder (_fs_folder_def fsfolder)
{
  
  if(_fs_search_free_sector(fsfolder.FolderSize) == ERROR){
    return ERROR;
  }
 
  return SUCCESS;
}

/**
* @brief  Searching for erased sectors to define a folder
* @param  hqspi : QSPI handle
* @retval HAL status
*/
static int _fs_search_free_sector (int size)
{
  #define _fs_search_free_sector_DEBUG      1
  
  volatile uint8_t i;
  uint8_t temp[20];
  int addressindex[10];
  int address = QSPI_START_ADDRESS;
  int sectorsize = (size / MX25R6435F_SECTOR_SIZE + 1);
  /*----------------------------------------------------------------------------*/
  
#if (_fs_search_free_sector_DEBUG == 1)  
  terminal("\nSectorSize: %d", sectorsize);
#endif 
  
  
  while(sectorsize){
    qspi_ReadMemory(temp, address, 10);
    
    
    
    #if (_fs_search_free_sector_DEBUG == 1)  
    terminal("\n0x%X_",address);
    #endif
    
    for(i = 0; i < 3; i++){
    
    #if (_fs_search_free_sector_DEBUG == 1)  
    terminal("%d_",temp[i]);
    #endif
      
      if(temp[i] != 0xFF){
        address += MX25R6435F_SECTOR_SIZE;
        i = 5;
      }
      if(address == 0x7FF000){
        terminal("\nError!\nEnd of SPI Flash, no free sector found!");
        return ERROR;
      }
    }
    if(i == 3){
      addressindex[sectorsize] = address;
      sectorsize--;
      address += MX25R6435F_SECTOR_SIZE;
    }
    if(i == 5){
      sectorsize = ((size / MX25R6435F_SECTOR_SIZE) + 1);
    }
    
  }
  
  terminal("\nADD\n");
  terminal("%X_", addressindex[0]);
  terminal("%X_", addressindex[1]);
  terminal("%X_", addressindex[2]);
  terminal("%X_", addressindex[3]);
  terminal("%X_", addressindex[4]);
  terminal("%X_", addressindex[5]);
  terminal("%X_", addressindex[6]);
  terminal("%X_", addressindex[7]);
  terminal("%X_", addressindex[8]);
  terminal("%X_", addressindex[9]);
  
  
  return SUCCESS;
}




/******************* (C) COPYRIGHT 2021 Farzin_M.Benam *****END OF FILE****/

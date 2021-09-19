#ifndef _FS_H
#define _FS_H

#ifdef __cplusplus
 extern "C" {
#endif



#include "main.h"


/*******************************************************************************
 * defintions
 *******************************************************************************/
#define   QSPI_START_ADDRESS  0x8000
#define		FS_FOLDER_START_TAG	
/* Exported constants --------------------------------------------------------*/


typedef struct{
  char		*buffer;
  uint8_t	length;
}fs_init_def;

typedef struct{
  char  Name[10];			// Name of the folder
  char  Commant[200]; // descreption of the folder
	int		StartAddr;		// starting address of the folder on spi flash
	int		EndAddr;			// end address of the folder on spi flash
	int		LastLogAddr;	// address of the last log
	uint8_t LogLength;
  

}fs_folder_def;



/* Exported functions --------------------------------------------------------*/ 
void	_fs_log (char *data, fs_folder_def *fsfolder);
ErrorStatus		_fs_new_folder	(fs_folder_def *fsfolder);
void		T_H_log_init (fs_folder_def *fsfolder);
void		T_H_log			(fs_folder_def *fsfolder);

#endif


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

typedef struct{
  uint8_t *buffer;
  int     length;
  uint8_t folder;
  
}_fs_init_def;

typedef struct{
  char  Name[10];
  int   FolderSize;   // per KB, smallest amount is 4KB, 
  char  Commant[200]; // descreption of the folder
  
  
}_fs_folder_def;



/* Exported functions --------------------------------------------------------*/ 
void  _fs_log (_fs_init_def fs);
ErrorStatus  _fs_new_folder (_fs_folder_def fsfolder);

#endif


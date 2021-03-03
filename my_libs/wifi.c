/* Includes ------------------------------------------------------------------*/
#include "wifi.h"


/* Private variables ---------------------------------------------------------*/
ES_WIFIObject_t    Obj;

extern volatile uint8_t	CmdStatus;
static uint8_t	CmdLenght;
extern volatile uint8_t	WifiStatus;
extern volatile uint8_t	wificmdStatus;

/* Private function prototypes -----------------------------------------------*/
static  uint8_t Hex2Num(char a);
static uint32_t ParseHexNumber(char* ptr, uint8_t* cnt);
static uint32_t ParseHexNumber(char* ptr, uint8_t* cnt);
static void ParseMAC(char* ptr, uint8_t* arr);
static void ParseIP(char* ptr, uint8_t* arr);
static ES_WIFI_SecurityType_t ParseSecurity(char* ptr);
#if (ES_WIFI_USE_UART == 1)
static void AT_ParseUARTConfig(char *pdata, ES_WIFI_UARTConfig_t *pConfig);
#endif
static void AT_ParseSystemConfig(char *pdata, ES_WIFI_SystemConfig_t *pConfig);
static void AT_ParseConnSettings(char *pdata, ES_WIFI_Network_t *NetSettings);
static ES_WIFI_Status_t AT_ExecuteCommand(ES_WIFIObject_t *Obj, uint8_t* cmd, uint8_t *pdata);
/************************************************************************
* wifi functions
************************************************************************/
/**
  * @brief  Join an Access Point
  * @param  SSID : SSID string
  * @param  Password : Password string
  * @param  ecn : Encryption type
  * @param  IP_Addr : Got IP Address
  * @param  IP_Mask : Network IP mask
  * @param  Gateway_Addr : Gateway IP address
  * @param  MAC : pointer to MAC Address
  * @retval Operation status
  */
WIFI_Status_t  WIFI_Connect(const char* SSID, const char* Password, WIFI_Ecn_t ecn)
{
//  WIFI_Status_t ret = WIFI_STATUS_ERROR;
  volatile int i;

  
  sprintf((char*)Obj.CmdData,"%s=%s\r",AT_NET_SET_SSID, SSID);
    
  
  for(i = 0 ;Obj.CmdData[i] != 0; i++){	// sending the command in buffer to ISM (wifi module)
    _usart3_send_b (Obj.CmdData[i]);
		_usart1_send_b (Obj.CmdData[i]);
	}
  _usart3_send_b (ENTER);				// MUST send an ENTER to apply the command in ISM module
//  ret = AT_ExecuteCommand(Obj, Obj->CmdData, Obj->CmdData);
  

  WifiStatus = 2;								// means now we expecting respons from ISM module
	CmdStatus = 0;								// because we coming from CmdEntry() so now exiting
	CmdLenght = 0;								// because we coming from CmdEntry() so now exiting
  
  wificmdStatus = 0;
	return 0;
}

///**
//  * @brief  Execute AT command.
//  * @param  Obj: pointer to module handle
//  * @param  cmd: pointer to command string
//  * @param  pdata: pointer to returned data
//  * @retval Operation Status.
//  */
//static ES_WIFI_Status_t AT_ExecuteCommand(ES_WIFIObject_t *Obj, uint8_t* cmd, uint8_t *pdata)
//{
// // if(Obj->fops.IO_Send(cmd, strlen((char*)cmd), Obj->Timeout) > 0)
//  if(_wifi_send("%s", cmd) > 0)
//  {
//    int16_t n=Obj->fops.IO_Receive(pdata, 0, Obj->Timeout);
//    if(n > 0)
//    {
//      *(pdata+n)=0;
//      if(strstr((char *)pdata, AT_OK_STRING))
//      {
//        return ES_WIFI_STATUS_OK;
//      }
//      else if(strstr((char *)pdata, AT_ERROR_STRING))
//      {
//        return ES_WIFI_STATUS_ERROR;
//      }
//    }
//  }
//  return ES_WIFI_STATUS_IO_ERROR;
//}

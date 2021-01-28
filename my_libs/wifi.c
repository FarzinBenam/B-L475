#include "wifi.h"

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
WIFI_Status_t  WIFI_Connect(const char* SSID,
                   const char* Password,
                   WIFI_Ecn_t ecn)
{
  WIFI_Status_t ret = WIFI_STATUS_ERROR;
  
  
}



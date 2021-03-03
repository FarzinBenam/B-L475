#ifndef __WIFI_H_
#define __WIFI_H_


#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "uart.h"
#include "wifi_cmds.h"
#include "Processes.h"

/* Exported constants --------------------------------------------------------*/
#define ES_WIFI_MAX_SSID_NAME_SIZE                  32
#define ES_WIFI_MAX_PSWD_NAME_SIZE                  32
#define ES_WIFI_PRODUCT_ID_SIZE                     32
#define ES_WIFI_PRODUCT_NAME_SIZE                   32
#define ES_WIFI_FW_REV_SIZE                         16
#define ES_WIFI_API_REV_SIZE                        16
#define ES_WIFI_STACK_REV_SIZE                      16
#define ES_WIFI_RTOS_REV_SIZE                       16

#define ES_WIFI_DATA_SIZE                           1600
#define ES_WIFI_MAX_DETECTED_AP                     10

#define ES_WIFI_TIMEOUT                             0xFFFFFF

#define ES_WIFI_USE_PING                            1
#define ES_WIFI_USE_AWS                             0
#define ES_WIFI_USE_FIRMWAREUPDATE                  0
#define ES_WIFI_USE_WPS                             0

#define ES_WIFI_USE_SPI                             1
#define ES_WIFI_USE_UART                            (!ES_WIFI_USE_SPI)


#define WIFI_MAX_SSID_NAME            100
#define WIFI_MAX_PSWD_NAME            100
#define WIFI_MAX_APS                  100
#define WIFI_MAX_CONNECTIONS          4
#define WIFI_MAX_MODULE_NAME          100
#define WIFI_MAX_CONNECTED_STATIONS   2
#define  WIFI_MSG_JOINED      1
#define  WIFI_MSG_ASSIGNED    2

/* Exported typedef ----------------------------------------------------------*/
typedef enum {
  ES_WIFI_STATUS_OK             = 0,
  ES_WIFI_STATUS_REQ_DATA_STAGE = 1,
  ES_WIFI_STATUS_ERROR          = 2,
  ES_WIFI_STATUS_TIMEOUT        = 3,
  ES_WIFI_STATUS_IO_ERROR       = 4,
}ES_WIFI_Status_t;

typedef enum {
  ES_WIFI_MODE_SINGLE           = 0,
  ES_WIFI_MODE_MULTI            = 1,
}ES_WIFI_ConnMode_t;

typedef enum {
  ES_WIFI_TCP_CONNECTION        = 0,
  ES_WIFI_UDP_CONNECTION        = 1,
  ES_WIFI_UDP_LITE_CONNECTION   = 2,
  ES_WIFI_TCP_SSL_CONNECTION    = 3,
  ES_WIFI_MQTT_CONNECTION       = 4,
}ES_WIFI_ConnType_t;

/* Security settings for wifi network */
typedef enum {
  ES_WIFI_SEC_OPEN = 0x00,          /*!< Wifi is open */
  ES_WIFI_SEC_WEP  = 0x01,          /*!< Wired Equivalent Privacy option for wifi security. \note This mode can't be used when setting up ES_WIFI wifi */
  ES_WIFI_SEC_WPA  = 0x02,          /*!< Wi-Fi Protected Access */
  ES_WIFI_SEC_WPA2 = 0x03,          /*!< Wi-Fi Protected Access 2 */
  ES_WIFI_SEC_WPA_WPA2= 0x04,       /*!< Wi-Fi Protected Access with both modes */
  ES_WIFI_SEC_WPA2_TKIP= 0x05,      /*!< Wi-Fi Protected Access with both modes */
  ES_WIFI_SEC_UNKNOWN = 0xFF,       /*!< Wi-Fi Unknown Security mode */
} ES_WIFI_SecurityType_t;

typedef enum {
  ES_WIFI_IPV4 = 0x00,
  ES_WIFI_IPV6 = 0x01,
} ES_WIFI_IPVer_t;

typedef enum {
  ES_WIFI_AP_NONE     = 0x00,
  ES_WIFI_AP_ASSIGNED = 0x01,
  ES_WIFI_AP_JOINED   = 0x02,
  ES_WIFI_AP_ERROR    = 0xFF,
} ES_WIFI_APState_t;


typedef struct
{
  uint32_t Port;
  uint32_t BaudRate;
  uint32_t DataWidth;
  uint32_t Parity;
  uint32_t StopBits;
  uint32_t Mode;

}ES_WIFI_UARTConfig_t;


typedef struct
{
  uint32_t Configuration;
  uint32_t WPSPin;
  uint32_t VID;
  uint32_t PID;
  uint8_t MAC[6];
  uint8_t AP_IPAddress[4];
  uint32_t PS_Mode;
  uint32_t RadioMode;
  uint32_t CurrentBeacon;
  uint32_t PrevBeacon;
  uint32_t ProductName;

}ES_WIFI_SystemConfig_t;

typedef struct {
  uint8_t* Address;                        /*!< Pointer to domain or IP to ping */
  uint32_t Time;                           /*!< Time in milliseconds needed for pinging */
  uint8_t Success;                         /*!< Status indicates if ping was successful */
}ES_WIFI_Ping_t;

typedef struct {
  uint8_t SSID[ES_WIFI_MAX_SSID_NAME_SIZE + 1]; /*!< Service Set Identifier value.Wi-Fi spot name */
  ES_WIFI_SecurityType_t Security;         /*!< Security of Wi-Fi spot.  */
  int16_t RSSI;                            /*!< Signal strength of Wi-Fi spot */
  uint8_t MAC[6];                          /*!< MAC address of spot */
  uint8_t Channel;                         /*!< Wi-Fi channel */
} ES_WIFI_AP_t;

/* Access point configuration */
typedef struct {
  uint8_t SSID[ES_WIFI_MAX_SSID_NAME_SIZE + 1];  /*!< Network public name for ESP AP mode */
  uint8_t Pass[ES_WIFI_MAX_PSWD_NAME_SIZE + 1];  /*!< Network password for ESP AP mode */
  ES_WIFI_SecurityType_t Security;          /*!< Security of Wi-Fi spot. This parameter can be a value of \ref ESP8266_Ecn_t enumeration */
  uint8_t Channel;                          /*!< Channel Wi-Fi is operating at */
  uint8_t MaxConnections;                   /*!< Max number of stations that are allowed to connect to ESP AP, between 1 and 4 */
  uint8_t Hidden;                           /*!< Set to 1 if network is hidden (not broadcast) or zero if noz */
} ES_WIFI_APConfig_t;


typedef struct {
  uint8_t SSID[ES_WIFI_MAX_SSID_NAME_SIZE + 1];  /*!< Network public name for ESP AP mode */
  uint8_t IP_Addr[4];                       /*!< IP Address */
  uint8_t MAC_Addr[6];                      /*!< MAC address */
} ES_WIFI_APSettings_t;

typedef struct {
  ES_WIFI_AP_t AP[ES_WIFI_MAX_DETECTED_AP];
  uint8_t nbr;

}ES_WIFI_APs_t;

typedef struct {
  uint8_t          SSID[ES_WIFI_MAX_SSID_NAME_SIZE + 1];
  uint8_t          pswd[ES_WIFI_MAX_PSWD_NAME_SIZE + 1];
  ES_WIFI_SecurityType_t Security;
  uint8_t          DHCP_IsEnabled;
  uint8_t          JoinRetries;
  uint8_t          IsConnected;
  uint8_t          AutoConnect;
  ES_WIFI_IPVer_t  IP_Ver;
  uint8_t          IP_Addr[4];
  uint8_t          IP_Mask[4];
  uint8_t          Gateway_Addr[4];
  uint8_t          DNS1[4];
  uint8_t          DNS2[4];
} ES_WIFI_Network_t;

#if (ES_WIFI_USE_AWS == 1)
typedef struct {
  ES_WIFI_ConnType_t Type;
  uint8_t            Number;
  uint16_t           RemotePort;
  uint8_t            RemoteIP[4];
  uint8_t            *PublishTopic;
  uint8_t            *SubscribeTopic;
  uint8_t            *ClientID;
  uint8_t            MQTTMode;
} ES_WIFI_AWS_Conn_t;
#endif

typedef struct {
  ES_WIFI_ConnType_t Type;
  uint8_t            Number;
  uint16_t           RemotePort;
  uint16_t           LocalPort;
  uint8_t            RemoteIP[4];
  char*              Name;
} ES_WIFI_Conn_t;


typedef struct {
  uint8_t                 Product_ID[ES_WIFI_PRODUCT_ID_SIZE];
  uint8_t                 FW_Rev[ES_WIFI_FW_REV_SIZE];
  uint8_t                 API_Rev[ES_WIFI_API_REV_SIZE];
  uint8_t                 Stack_Rev[ES_WIFI_STACK_REV_SIZE];
  uint8_t                 RTOS_Rev[ES_WIFI_RTOS_REV_SIZE];
  uint8_t                 Product_Name[ES_WIFI_PRODUCT_NAME_SIZE];
  uint32_t                CPU_Clock;
  ES_WIFI_SecurityType_t  Security;
  ES_WIFI_Network_t       NetSettings;
  ES_WIFI_APSettings_t    APSettings;
  uint8_t                 CmdData[ES_WIFI_DATA_SIZE];
  uint32_t                Timeout;
  uint32_t                BufferSize;
}ES_WIFIObject_t;

/* Exported types ------------------------------------------------------------*/
typedef enum {
  WIFI_ECN_OPEN = 0x00,
  WIFI_ECN_WEP = 0x01,
  WIFI_ECN_WPA_PSK = 0x02,
  WIFI_ECN_WPA2_PSK = 0x03,
  WIFI_ECN_WPA_WPA2_PSK = 0x04,
}WIFI_Ecn_t;

typedef enum {
  WIFI_TCP_PROTOCOL = 0,
  WIFI_UDP_PROTOCOL = 1,
}WIFI_Protocol_t;

typedef enum {
  WIFI_SERVER = 0,
  WIFI_CLIENT = 1,
}WIFI_Type_t;

typedef enum {
  WIFI_STATUS_OK             = 0,
  WIFI_STATUS_ERROR          = 1,
  WIFI_STATUS_NOT_SUPPORTED  = 2,
  WIFI_STATUS_JOINED         = 3,
  WIFI_STATUS_ASSIGNED       = 4,
}WIFI_Status_t;

typedef struct {
  WIFI_Ecn_t Ecn;                                           /*!< Security of Wi-Fi spot. This parameter has a value of \ref WIFI_Ecn_t enumeration */
  char SSID[WIFI_MAX_SSID_NAME + 1];                        /*!< Service Set Identifier value. Wi-Fi spot name */
  int16_t RSSI;                                             /*!< Signal strength of Wi-Fi spot */
  uint8_t MAC[6];                                           /*!< MAC address of spot */
  uint8_t Channel;                                          /*!< Wi-Fi channel */
  uint8_t Offset;                                           /*!< Frequency offset from base 2.4GHz in kHz */
  uint8_t Calibration;                                      /*!< Frequency offset calibration */
}WIFI_AP_t;

typedef struct {
  WIFI_AP_t    ap[WIFI_MAX_APS];
  uint8_t      count;
} WIFI_APs_t;


typedef struct {
  uint8_t Number;                                           /*!< Connection number */
  uint16_t RemotePort;                                      /*!< Remote PORT number */
  uint16_t LocalPort;
  uint8_t RemoteIP[4];                                      /*!< IP address of device */
  WIFI_Protocol_t Protocol;                                 /*!< Connection type. Parameter is valid only if connection is made as client */
  uint32_t TotalBytesReceived;                              /*!< Number of bytes received in entire connection lifecycle */
  uint32_t TotalBytesSent;                                  /*!< Number of bytes sent in entire connection lifecycle */
  uint8_t Active;                                           /*!< Status if connection is active */
  uint8_t Client;                                           /*!< Set to 1 if connection was made as client */
} WIFI_Socket_t;


typedef struct {

  uint8_t          SSID[WIFI_MAX_SSID_NAME + 1];
  uint8_t          PSWD[WIFI_MAX_PSWD_NAME + 1];
  uint8_t          channel;
  WIFI_Ecn_t       Ecn;
} WIFI_APConfig_t;

typedef struct {
  uint8_t SSID[WIFI_MAX_SSID_NAME + 1];                         /*!< Network public name for ESP AP mode */
  uint8_t IP_Addr[4];                                           /*!< IP Address */
  uint8_t MAC_Addr[6];                                          /*!< MAC address */
} WIFI_APSettings_t;

typedef struct {
  uint8_t          IsConnected;
  uint8_t          IP_Addr[4];
  uint8_t          IP_Mask[4];
  uint8_t          Gateway_Addr[4];
} WIFI_Conn_t;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
WIFI_Status_t   WIFI_Init(void);
WIFI_Status_t   WIFI_ListAccessPoints(WIFI_APs_t *APs, uint8_t AP_MaxNbr);
WIFI_Status_t   WIFI_Connect(const char* SSID, const char* Password, WIFI_Ecn_t ecn);
WIFI_Status_t   WIFI_GetIP_Address(uint8_t  *ipaddr);
WIFI_Status_t   WIFI_GetMAC_Address(uint8_t  *mac);

WIFI_Status_t   WIFI_Disconnect(void);
WIFI_Status_t   WIFI_ConfigureAP(	uint8_t *ssid,
                                  uint8_t *pass,
                                  WIFI_Ecn_t ecn,
                                  uint8_t channel,
                                  uint8_t max_conn);

WIFI_Status_t   WIFI_HandleAPEvents(WIFI_APSettings_t *setting);
WIFI_Status_t   WIFI_Ping(uint8_t* ipaddr, uint16_t count, uint16_t interval_ms);
WIFI_Status_t   WIFI_GetHostAddress( char* location, uint8_t* ipaddr);
WIFI_Status_t   WIFI_OpenClientConnection(uint32_t socket, WIFI_Protocol_t type, const char* name, uint8_t* ipaddr, uint16_t port, uint16_t local_port);
WIFI_Status_t   WIFI_CloseClientConnection(uint32_t socket);

WIFI_Status_t   WIFI_StartServer(uint32_t socket, WIFI_Protocol_t type, const char* name, uint16_t port);
WIFI_Status_t   WIFI_StopServer(uint32_t socket);

WIFI_Status_t   WIFI_SendData(uint8_t socket, uint8_t *pdata, uint16_t Reqlen, uint16_t *SentDatalen, uint32_t Timeout);
WIFI_Status_t   WIFI_ReceiveData(uint8_t socket, uint8_t *pdata, uint16_t Reqlen, uint16_t *RcvDatalen, uint32_t Timeout);
WIFI_Status_t   WIFI_StartClient(void);
WIFI_Status_t   WIFI_StopClient(void);

WIFI_Status_t   WIFI_SetOEMProperties(const char *name, uint8_t *Mac);
WIFI_Status_t   WIFI_ResetModule(void);
WIFI_Status_t   WIFI_SetModuleDefault(void);
WIFI_Status_t   WIFI_ModuleFirmwareUpdate(const char *url);
WIFI_Status_t   WIFI_GetModuleID(char *Id);
WIFI_Status_t   WIFI_GetModuleFwRevision(char *rev);
WIFI_Status_t   WIFI_GetModuleName(char *ModuleName);
WIFI_Status_t   WIFI_GetCredentials(uint8_t *ssid, uint8_t *password, WIFI_Ecn_t *security);
#ifdef __cplusplus
}
#endif

#endif /* __WIFI_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

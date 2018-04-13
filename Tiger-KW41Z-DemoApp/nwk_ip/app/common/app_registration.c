//////////////////////////////////////////////////////////////
// Copyright(c) 2017, Volansys Technologies
//
// Description:
/// \file app_registration.c
/// \brief This is the source file for ED registration.
///
//
// Author Volansys Technologies
//
//////////////////////////////////////////////////////////////

/*==================================================================================================
Include Files
==================================================================================================*/
#include "app_registration.h"
#include <string.h>
#include <stdio.h>

/* FSL Framework */
#include "MemManager.h"
#include "FunctionLib.h"
#include "fsl_os_abstraction.h"

/* Application */
#include "sockets.h"
#include "app_thread_config.h"
#include "session.h"

#include "coap.h"
#include "coap_cfg.h"
#include "ip_if_management.h"
#include "thread_utils.h"

#if THREAD_USE_SHELL
    #include "shell.h"
#endif

#if STACK_THREAD
#include "thread_network.h"
#include "thread_utils.h"
#include "mle_basic.h"
#include "app_thread_config.h"
#endif

#if THREAD_USE_THCI
    #include "thci.h"
    #include "FsciInterface.h"
#endif

/*==================================================================================================
Private macros
==================================================================================================*/
#define TEST_SERVER_PORT    (3002U)

#define THREAD_DEVICE_VERSION   gOtaCurrentFileVersionNo_c
#if RGB_CLICK_ENABLE && VT_KW41Z_MENP
#define THREAD_DEVICE_MENPv1_MODEL    "MENPv1-KW41Z-RGBCLICK"
#define THREAD_DEVICE_MENPv2_MODEL    "MENPv2-KW41Z-RGBCLICK"
#elif RELAY_CLICK_ENABLE && VT_KW41Z_MENP
#define THREAD_DEVICE_MENPv1_MODEL    "MENPv1-KW41Z-RELAYCLICK"
#define THREAD_DEVICE_MENPv2_MODEL    "MENPv2-KW41Z-RELAYCLICK"
#elif VT_KW41Z_DEMO
#define THREAD_DEVICE_MODEL     "FRDM-KW41Z-RGBLED"
#endif

#define DEVICE_ONLINE_CMD_MAX_SIZE      120
#define FIXED_TLV_SIZE      (2U) /* 1 byte tag + 1 byte length */
#define DEVICE_ONLINE_CMD   (0x0001)
/*==================================================================================================
Private type definitions
==================================================================================================*/
typedef enum
{
  TAG_CMD_ID = 0x10,
  TAG_EUI,
  TAG_IP,
  TAG_PROFILE_LIST,
  TAG_FIRMWARE_VERSION,
  TAG_MODEL_NAME
} onlineMsgTag;

/*==================================================================================================
Private prototypes
==================================================================================================*/
void Task1(osaTaskParam_t argument);

/*==================================================================================================
Global variables declarations
==================================================================================================*/
#if VT_KW41Z_MENP
extern bool_t gIsOTASupported;
#endif /* #if VT_KW41Z_MENP */

/*==================================================================================================
Private global variables declarations
==================================================================================================*/
OSA_TASK_DEFINE(Task1, OSA_PRIORITY_IDLE,  1, 1024, 0);
osaTaskId_t taskId1;
uint8_t registration_sucessful = 0;
static int32_t mtestUdpSrvSockFd = -1;

/*==================================================================================================
Public global variables declarations
==================================================================================================*/
void App_Task_Init(void)
{
  taskId1 = OSA_TaskCreate(OSA_TASK(Task1), (osaTaskParam_t)NULL);
}

void MYSHELL_PrintBuff
(
    uint8_t * buff,
    uint32_t length
)
{
    uint32_t i;

    for(i = 0; i < length; i++)
    {
        shell_printf("%02X",buff[i]);
    }
}

uint8_t addToTLV_packet(onlineMsgTag tag,
                       uint8_t *destBuf, uint8_t *destBufLen,
                       const void *srcBuf, uint8_t srcBufLen)
{
  uint8_t numCopy = 0;
  if ((destBuf != NULL) && (destBufLen != NULL) && (*destBufLen > 0) && (srcBuf != NULL) && (srcBufLen > 0))
  {
    if (*destBufLen >= (srcBufLen + FIXED_TLV_SIZE))
    {
      destBuf[0] = (uint8_t) tag;
      destBuf[1] = srcBufLen;
      memcpy(&destBuf[2], srcBuf, srcBufLen);
      numCopy = (srcBufLen + FIXED_TLV_SIZE);
      *destBufLen -= numCopy;
    }
    else
    {
      shell_printf("Payload buffer full\r\n");
    }
  }
  
  if (numCopy == 0)
  {
    shell_printf("addToTLV_packet failed for Tag 0x%02x\r\n", (uint8_t) tag);
  }
  
  return numCopy;
}

uint8_t create_buffer(uint8_t *data)
{
  uint8_t eui_value[8];
  uint16_t tempInt;
  uint8_t remainLen = DEVICE_ONLINE_CMD_MAX_SIZE;
  
  /* Command Id */
  tempInt = htons(DEVICE_ONLINE_CMD);
  data += addToTLV_packet(TAG_CMD_ID, data, &remainLen, &tempInt, sizeof(uint16_t));
  
  /* EUI of the device */
  /* Get own eui */
  uint32_t attrLength = 0;
  THR_GetAttr(0,gNwkAttrId_IeeeAddr_c, 0, &(attrLength), &eui_value);
  NWKU_SwapArrayBytes((uint8_t*)&eui_value, 8);
  
  data += addToTLV_packet(TAG_EUI, data, &remainLen, eui_value, 8);
  
  /* Get own ULA */
  uint32_t iIfIdx, iIpIdx;
  char addrStr[INET6_ADDRSTRLEN];
  ifHandle_t pIfHandle;
  ip6IfAddrData_t *pIp6AddrData;
  /* IP Interfaces */
  for(iIfIdx = 0; iIfIdx < IP_IF_NB; iIfIdx++)
  {
    /* Get interface by index */
    pIfHandle = IP_IF_GetIfByIndex(iIfIdx);
    
    if(pIfHandle)
    {
      /* Get IPv6 addresses for an interface */
      for(iIpIdx = 0; iIpIdx < IP_IF_IP6_ADDR_NB; iIpIdx++)
      {
        /* get IP address by index */
        pIp6AddrData = IP_IF_GetAddrByIf6(pIfHandle->ifUniqueId, iIpIdx, 0);
        
        if (pIp6AddrData)
        {
          ntop(AF_INET6, &pIp6AddrData->ip6Addr, addrStr, INET6_ADDRSTRLEN);
          
          if (IP6_IsUniqueLocalAddr(&pIp6AddrData->ip6Addr))
          {
            thrPrefixAttr_t thrMLPrefix;
            uint32_t MLPrefixLen = sizeof(ipAddr_t);
            
            (void)THR_GetAttr(0, gNwkAttrId_MLPrefix_c, 0, &(MLPrefixLen), &(thrMLPrefix));
            if (FLib_MemCmp(&pIp6AddrData->ip6Addr,&thrMLPrefix.prefix,8))
            {
              /* This is ML64 address */
            }
            else
            {
              uint8_t profileList[25];
              uint8_t profileListLen = 0;
              uint32_t firmwareVersion;
              
              shell_printf("My Unique local address: %s\r\n", addrStr);
              
              /* IPv6 address */
              data += addToTLV_packet(TAG_IP, data, &remainLen, &(pIp6AddrData->ip6Addr), 16);
              
              /* Profile list */
              memset(profileList, 0x00, sizeof(profileList));
#if ACCELEROMETER_ENABLE && VT_KW41Z_DEMO
              tempInt = Accelerometer;
              memcpy(&profileList[profileListLen], &tempInt, sizeof(uint16_t));
              profileListLen += sizeof(uint16_t);
#endif

#ifndef RELAY_CLICK_ENABLE              
#if VT_KW41Z_MENP && RGB_CLICK_ENABLE
              tempInt = LED;
              memcpy(&profileList[profileListLen], &tempInt, sizeof(uint16_t));
              profileListLen += sizeof(uint16_t);
#else
              tempInt = RGBLed;
              memcpy(&profileList[profileListLen], &tempInt, sizeof(uint16_t));
              profileListLen += sizeof(uint16_t);
#endif /* VT_KW41Z_MENP */
#endif /* RELAY_CLICK_ENABLE */
              
              tempInt = Keycount;
              memcpy(&profileList[profileListLen], &tempInt, sizeof(uint16_t));
              profileListLen += sizeof(uint16_t);
#if RGB_CLICK_ENABLE && VT_KW41Z_MENP              
              tempInt = RGBClick;
              memcpy(&profileList[profileListLen], &tempInt, sizeof(uint16_t));
              profileListLen += sizeof(uint16_t);
#elif RELAY_CLICK_ENABLE && VT_KW41Z_MENP
              tempInt = RelayClick;
              memcpy(&profileList[profileListLen], &tempInt, sizeof(uint16_t));
              profileListLen += sizeof(uint16_t);
#endif
              
              data += addToTLV_packet(TAG_PROFILE_LIST, data, &remainLen, profileList, profileListLen);
              
              /* Firmware Version */
              firmwareVersion = THREAD_DEVICE_VERSION;
              data += addToTLV_packet(TAG_FIRMWARE_VERSION, data, &remainLen, &firmwareVersion, sizeof(uint32_t));
              
              /* Model Name */
#if VT_KW41Z_MENP
              if(gIsOTASupported)
              {
                data += addToTLV_packet(TAG_MODEL_NAME, data, &remainLen, THREAD_DEVICE_MENPv2_MODEL, strlen(THREAD_DEVICE_MENPv2_MODEL));
              }
              else
              {
                data += addToTLV_packet(TAG_MODEL_NAME, data, &remainLen, THREAD_DEVICE_MENPv1_MODEL, strlen(THREAD_DEVICE_MENPv1_MODEL));
              }
#else
              data += addToTLV_packet(TAG_MODEL_NAME, data, &remainLen, THREAD_DEVICE_MODEL, strlen(THREAD_DEVICE_MODEL));
#endif /* VT_KW41Z_MENP */
              
              shell_printf("Online Packet Size : %u\r\n", (DEVICE_ONLINE_CMD_MAX_SIZE - remainLen));
              
              return (DEVICE_ONLINE_CMD_MAX_SIZE - remainLen);
            }
          }
        }
      }
    }
  }
  return (DEVICE_ONLINE_CMD_MAX_SIZE - remainLen);
}

void udp_service(void *pInData)
{
    sessionPacket_t *pSessionPacket = (sessionPacket_t*)pInData;
    static char ipAddr[INET6_ADDRSTRLEN];
#if THREAD_USE_SHELL
    ntop(AF_INET6, &(pSessionPacket->remAddr.ss_addr), ipAddr, INET6_ADDRSTRLEN);
    shell_printf("Message received from %s: bytes=%d, time=", ipAddr, pSessionPacket->dataLen);
    shell_refresh();
    shell_write(ipAddr);
#endif
    registration_sucessful = 1;
    MEM_BufferFree(pSessionPacket->pData);
    MEM_BufferFree(pSessionPacket);
}


void Task1(osaTaskParam_t argument)
{
  sockaddrStorage_t portAddr;
  ipAddr_t pDestAddr;
  uint8_t payload[DEVICE_ONLINE_CMD_MAX_SIZE];
  int32_t packetSize = 0;
  int32_t packet_count = 0;
  
  pDestAddr.addr8[0]=0xFD;
  pDestAddr.addr8[1]=0x01;
  pDestAddr.addr8[2]=0x00;
  pDestAddr.addr8[3]=0x00;
  pDestAddr.addr8[4]=0x00;
  pDestAddr.addr8[5]=0x00;
  pDestAddr.addr8[6]=0x00;
  pDestAddr.addr8[7]=0x00;
  pDestAddr.addr8[8]=0x00;
  pDestAddr.addr8[9]=0x00;
  pDestAddr.addr8[10]=0x00;
  pDestAddr.addr8[11]=0x00;
  pDestAddr.addr8[12]=0x00;
  pDestAddr.addr8[13]=0x00;
  pDestAddr.addr8[14]=0x00;
  pDestAddr.addr8[15]=0x02;
  
  shell_printf("Task Created\r\n");
  /* Check if the socket has already been created */
  if (mtestUdpSrvSockFd == -1)
  {
    /* Set local address and local port */
    FLib_MemSet(&portAddr,0,sizeof(sockaddrStorage_t));

    /* Open UDP socket */
    mtestUdpSrvSockFd = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
    bind(mtestUdpSrvSockFd,&portAddr,sizeof(sockaddrStorage_t));
    ((sockaddrIn6_t*)&portAddr)->sin6_family = AF_INET6;
    ((sockaddrIn6_t*)&portAddr)->sin6_port = TEST_SERVER_PORT;
    IP_AddrCopy(&((sockaddrIn6_t*)&portAddr)->sin6_addr, &pDestAddr);
    Session_RegisterCb(mtestUdpSrvSockFd, udp_service, NULL);
  }
  
  while(!registration_sucessful)
  {
    packetSize = create_buffer(&payload[0]);
    shell_printf("Sending Buffer : ");
    MYSHELL_PrintBuff(&payload[0], packetSize);
    shell_printf("\n\r");
    if(packetSize > 8)
    {
      uint32_t secLevel = 5; ///TODO: Should come from parser.
      ((sockaddrIn6_t*)&portAddr)->sin6_flowinfo = BSDS_SET_TX_SEC_FLAGS(1, secLevel);
      sendto(mtestUdpSrvSockFd,(void*)&payload[0], packetSize, 0, &portAddr, sizeof(sockaddrStorage_t));
      packet_count++;
    }
    if(packet_count > 100)
    {
      ResetMCU();
    }
    OSA_TimeDelay(1000);
  }
  
  Session_UnRegisterCb(mtestUdpSrvSockFd);
  shutdown(mtestUdpSrvSockFd,0);
  while(1) {
    OSA_TimeDelay(10000);
  }
}

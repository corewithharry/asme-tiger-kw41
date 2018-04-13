//////////////////////////////////////////////////////////////
// Copyright(c) 2016, Volansys Technologies
//
// Description:
/// \file app_Relay.c
/// \brief This is the source file for mikroBus Relay configuration.
///
//
// Author Volansys Technologies
//
//////////////////////////////////////////////////////////////

#if RELAY_CLICK_ENABLE && VT_KW41Z_MENP

/*==================================================================================================
Include Files
==================================================================================================*/
#include "app_Relay.h"

#define RELAY_POST_PAYLOAD_FORMAT       "relay:%d,state:%d"
#define RELAY_DATA_BUFFER               50
#define MAX_SUPPORTED_RELAY             2

static void Relay_GpioSetReset(uint32_t relayNum, int32_t operation);
/******************************************************************************
* Name: Relay_Init
* Description: Initialize the Relay module
* Parameters: - None
* Return: - None
******************************************************************************/
void Relay_Init(void)
{
  uint8_t i=0;
  
  /* Initialize KBD pins. Function alse set pin MUX as GPIO */ 
  for(i=0;i<MAX_SUPPORTED_RELAY;i++)
  {
    GpioOutputPinInit (&realyPins[i],1);
  }
}

/******************************************************************************
* Name: APP_CoapRelayCb
* Description: Initialize the Relay module
* Parameters: - CoAP Session status, Data, CoAP Session structure, Data length
* Return: - None
******************************************************************************/
void APP_CoapRelayCb(coapSessionStatus_t sessionStatus, void *pData, 
                            coapSession_t *pSession, uint32_t dataLen)
{
    char sendRelayData[RELAY_DATA_BUFFER];
    char *pTempString = NULL;
    uint32_t ackPloadSize = 0;
    uint16_t index=0;
    int32_t relayNum, relayStatus;
    
    shell_write("RELAY : \r\n");
    
    memset(sendRelayData,0,RELAY_DATA_BUFFER);
    if(gCoapGET_c == pSession->code)
    {
        sprintf(sendRelayData, "{\"Rly\":[{\"N\":1,\"V\":%d},{\"N\":2,\"V\":%d}]}", GpioReadOutputPin(&realyPins[0]), GpioReadOutputPin(&realyPins[1]));
        pTempString = &sendRelayData[0];
        ackPloadSize = strlen((char*)pTempString);

        if (gCoapConfirmable_c == pSession->msgType)
        {
            if (gCoapGET_c == pSession->code)
            {
                COAP_Send(pSession, gCoapMsgTypeAckSuccessChanged_c, pTempString, ackPloadSize);
            }
            else
            {
                COAP_Send(pSession, gCoapMsgTypeAckSuccessChanged_c, NULL, 0);
            }
        }
    }
    else if((sessionStatus == gCoapSuccess_c) && (pSession->code == gCoapPOST_c))
    {
        shell_write("RELAY Set: ");
        sscanf((char *)pData, RELAY_POST_PAYLOAD_FORMAT, &relayNum, &relayStatus);
        shell_printf("%d  %d\r\n", relayNum, relayStatus);
        Relay_GpioSetReset(relayNum, relayStatus);
        
        memcpy(sendRelayData, pData, dataLen);
        pTempString = &sendRelayData[0];
        ackPloadSize = strlen((char*)pTempString);
       /* Send the reply if the status is Success or Duplicate */
        if ((gCoapFailure_c != sessionStatus) && (gCoapConfirmable_c == pSession->msgType))
        {
            /* Send CoAP ACK */
            COAP_Send(pSession, gCoapMsgTypeAckSuccessChanged_c, pTempString, ackPloadSize);
        }
    }
}

/******************************************************************************
* Name: Relay_GpioSetReset
* Description: Initialize the Relay module
* Parameters: - Relay identification number, Operation state
* Return: - None
******************************************************************************/
static void Relay_GpioSetReset(uint32_t relayNum, int32_t operation)
{
    switch(operation)
    {
        case 0:
            GpioClearPinOutput(&realyPins[relayNum - 1]);
            break;
            
        case 1:
            GpioSetPinOutput(&realyPins[relayNum - 1]);
            break;

        default:
            break;
    }
}

#endif


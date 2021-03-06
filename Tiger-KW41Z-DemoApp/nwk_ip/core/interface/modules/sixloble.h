/*
 * Copyright (c) 2014 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _SIXLOBLE_H
#define _SIXLOBLE_H
/*!=================================================================================================
\file       sixloble.h
\brief      This is a header file for the 6LoBLE module.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/

/* General Includes */
#include "EmbeddedTypes.h"

#include "TimersManager.h"

#include "stack_config.h"

#include "network_utils.h"
#include "ble_abs_types.h"
#include "sixloble_interface.h"
#include "sixlowpan.h"
#include "sixlowpan_ib.h"

/*==================================================================================================
Public macros
==================================================================================================*/



/*==================================================================================================
Public type definitions
==================================================================================================*/

typedef struct sloBlePktInfo_tag
{
    ipAddr_t                ipDstAddr;              /*!< Destination (Next Hop) IP Address
                                                      If Not in6addr_any then the IP stack provides the next hop IP address.
                                                      If in6addr_any then 6LoWPAN computes the next hop address from the IP header.*/
    instanceId_t            instanceId;            /* 6LoWPAN instance Id */                                              
    bleDataReq_t *          pBleDataReq;           /* Pointer to BleDataReq structure that will be passed to MAC */
    compressionInfo_t       compressInfo;           /* Compression information */
    transmissionType_t      transmissionType;       /* Unicast / Multicast / Broadcast */
    adpPacketType_t         packetType;             /* Type of Packet: IPv6, LBP, Mesh Routing, etc */    
    uint16_t                adpPayloadSize;
    nwkBuffer_t             adpPayload;             /* Original ADP Payload from upper layers
                                                       (IPv6, LBP, MESH) */
    nwkBuffer_t             compressedHeader;       /*  Compressed Header */
} sloBlePktInfo_t;


/*==================================================================================================
Public global variables declarations
==================================================================================================*/

extern taskMsgQueue_t  mSlwpMsgQueue;
extern taskMsgQueue_t* pSlwpMsgQueue;

extern sloBleStruct_t * mpSloBleStruct;
/*==================================================================================================
Public function prototypes
==================================================================================================*/
#ifdef __cplusplus
extern "C" {
#endif

/*!*************************************************************************************************
\fn     void SixLoBle_HandleAdpdDataReq(void * pPayload)
\brief  Function to handle the ADPD Data Request message.

\param  [in]    pPayload       Pointer to the ADPD Data Request structure.
***************************************************************************************************/
void SixLoBle_HandleAdpdDataReq(void * pPayload);

/*!*************************************************************************************************
\fn     void SixLoBle_HandleBleDataInd(void *pPayload)
\brief  Function used to handle the received BLE Data Indication.

\param  [in]    pPayload    Pointer to the received BLE Data Indication.
***************************************************************************************************/
void SixLoBle_HandleBleDataInd(void * pPayload);

/*!*************************************************************************************************
\fn     void SixLoBle_FreePktInfo(slwpPktInfo_t * pSlwpPktInfo)
\brief  Function used to free the 6LoWPAN packet information.

\param  [in]    pSlwpPktInfo      Pointer to the 6Lo packet information structure.
***************************************************************************************************/
void SixLoBle_FreePktInfo(sloBlePktInfo_t *pSloPktInfo);

/*!*************************************************************************************************
\fn     void SixLoBle_DropPacket(bleDataInd_t * pBleDataInd)
\brief  Function used to free the MCPS Data Indication.

\param  [in]    pBleDataInd      Pointer to the MCPS Data Indication structure.
***************************************************************************************************/
void SixLoBle_DropPacket(bleDataInd_t *pBleDataInd);

/*!*************************************************************************************************
\fn     void SixLoBle_GenerateAdpdDataInd(macAbsBleDataInd_t * pBleDataInd)
\brief  Function used to generate a ADPD Data Indication from a received MCPS Data Indication.

\param  [in]    pBleDataInd  Pointer to the MCPS Data Indication (MUST NOT be NULL)
***************************************************************************************************/
void SixLoBle_GenerateAdpdDataInd(bleDataInd_t * pBleDataInd);

#ifdef __cplusplus
}
#endif
/*================================================================================================*/


#endif  /*_SIXLOBLE_H */

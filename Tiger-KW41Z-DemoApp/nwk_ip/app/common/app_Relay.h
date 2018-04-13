//////////////////////////////////////////////////////////////
// Copyright(c) 2016, Volansys Technologies
//
// Description:
/// \file app_Relay.h
/// \brief This is the header file for mikroBus Relay configuration.
///
//
// Author Volansys Technologies
//
//////////////////////////////////////////////////////////////

#ifndef _APP_MIKROBUS_H             
#define _APP_MIKROBUS_H

#if RELAY_CLICK_ENABLE
/*==================================================================================================
Include Files
==================================================================================================*/
#include "board.h"

#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_os_abstraction.h"

#include "GPIO_Adapter.h"
#include "gpio_pins.h"

#include "app_stack_config.h"
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
#include "shell.h"

void Relay_Init(void);
void APP_CoapRelayCb(coapSessionStatus_t sessionStatus, void *pData, 
                            coapSession_t *pSession, uint32_t dataLen);

#endif /* RELAY_CLICK_ENABLE */

#endif

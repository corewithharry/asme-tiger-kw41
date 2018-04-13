//////////////////////////////////////////////////////////////
// Copyright(c) 2016, Volansys Technologies
//
// Description:
/// \file app_Relay.h
/// \brief This is the header file for mikroBus Relay configuration.
///
//
// Author Tushar Rabadiya
//
//////////////////////////////////////////////////////////////

#ifndef _APP_RGBCLICK_H             
#define _APP_RGBCLICK_H

#if RGB_CLICK_ENABLE
/*==================================================================================================
Include Files
==================================================================================================*/
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

#include "board.h"

#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_os_abstraction.h"

#include "GPIO_Adapter.h"
#include "gpio_pins.h"

#include "app_stack_config.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/*==================================================================================================
MACROS AND DEFINITIONS
==================================================================================================*/

//LEDs on borad
#define MAX_SIZE  16

//Define default colors
#define WHITE_Color     0x002F2F2F
#define RED_Color       0x00330000
#define GREEN_Color     0x00003300
#define BLUE_Color      0x00000033
#define ORANGE_Color    0x00333300

//Define low bright colors
#define RED_Low_Bright       0x00200000
#define GREEN_Low_Bright     0x00002000
#define BLUE_Low_Bright      0x00000020
#define WHITE_Low_Bright     0x00202020

//Define medium bright colors
#define RED_MED_Bright       0x00660000
#define GREEN_MED_Bright     0x00006600
#define BLUE_MED_Bright      0x00000066
#define WHITE_MED_Bright     0x00666666
#define ORANGE_MED_Color     0x00666600

//Define high bright colors
#define RED_HIGH_Bright      0x00990000
#define GREEN_HIGH_Bright    0x00009900
#define BLUE_HIGH_Bright     0x00000099
#define WHITE_HIGH_Bright    0x00999999

// Max allowed brightness
#define MAX_ALLOWED_BRIGHTNESS  0x99

#define RGBCLICK_URI_PATH                      "/rgbclick"
#define RGBCLICK_URI_PATH_LEN                      9
#define RGBColorOperation                         0x00

/*==================================================================================================
Public function prototypes
==================================================================================================*/
void RGBLed_Init(void);
void Snake(unsigned long);
void FillScreen(unsigned long);
void BlankScreen();
void Snake_return (unsigned long);
void APP_CoapRGBCb(coapSessionStatus_t sessionStatus, void *pData, 
                            coapSession_t *pSession, uint32_t dataLen);

#endif /* RGB_CLICK_ENABLE */
#endif /* _APP_RGBCLICK_H */

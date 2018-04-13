//////////////////////////////////////////////////////////////
// Copyright(c) 2016, Volansys Technologies
//
// Description:
/// \file app_accelerometer.c
/// \brief This is a header file for the application accelerometer data reading.
///
//
// Author Volansys Technologies
//
//////////////////////////////////////////////////////////////

#ifndef _APP_ACCELEROMETER_H             
#define _APP_ACCELEROMETER_H

/*!=================================================================================================
\file       app_led.h
\brief      This is the header file for the application led configuration
==================================================================================================*/
#include <stdio.h>
#include "shell.h"
#include "FunctionLib.h"
#include "coap.h"
#include "fsl_os_abstraction.h"

/*==================================================================================================
User defines
==================================================================================================*/
#define ACCELEROMETER_URI_PATH                      "/acc"
#define ACCELEROMETER_URI_PATH_LEN                      4
/*==================================================================================================
Exported APIs
==================================================================================================*/
void accelerometerInit();

void Accelerometer_CoapCb
(
    coapSessionStatus_t sessionStatus,
    void *pData,
    coapSession_t *pSession,
    uint32_t dataLen
);

void App_GetAccDataString(char *data);
//void accelerometerCoAPInit();

#endif

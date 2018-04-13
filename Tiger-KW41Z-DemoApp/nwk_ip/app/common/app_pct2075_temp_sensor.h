#ifndef _APP_PCT2075_H
#define _APP_PCT2075_H

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
//#define ACCELEROMETER_URI_PATH                      "/acc"
//#define ACCELEROMETER_URI_PATH_LEN                      4
/*==================================================================================================
Exported APIs
==================================================================================================*/

int16_t pct2075_read_temperature(void);
void pct2075_get_temperature_string(char *datastr);

#endif //_APP_PCT2075_H

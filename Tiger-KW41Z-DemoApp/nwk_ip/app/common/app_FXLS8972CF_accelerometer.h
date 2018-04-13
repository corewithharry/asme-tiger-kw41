#ifndef _APP_FXLS8972_ACCELEROMETER_H
#define _APP_FXLS8972_ACCELEROMETER_H

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
/*==================================================================================================
Exported APIs
==================================================================================================*/
void accelerometer_FXLS8972_Init();
void accelerometer_FXLS8972_get_data_string(char *data);

#endif //_APP_FXLS8972_ACCELEROMETER_H

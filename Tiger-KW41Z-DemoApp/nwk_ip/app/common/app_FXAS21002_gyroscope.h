#ifndef _APP_FXAS21002_GYROSCOPE_H
#define _APP_FXAS21002_GYROSCOPE_H

/*!=================================================================================================
\file       app_led.h
\brief      This is the header file for the application led configuration
==================================================================================================*/
#include <stdio.h>
#include "shell.h"
#include "FunctionLib.h"
#include "coap.h"
#include "fsl_os_abstraction.h"

uint8_t gyroscope_init(void);
uint8_t gyroscope_read_data(int16_t* panData);
void gyroscope_get_data_string(char *datastr);
uint8_t gyroscope_read_temperature(int8_t* pnTemp);

#endif //_APP_FXAS21002_GYROSCOPE_H

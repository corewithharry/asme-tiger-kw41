#ifndef _APP_GPIO_EXPANDER_H
#define _APP_GPIO_EXPANDER_H

/*!=================================================================================================
\file       app_led.h
\brief      This is the header file for the application led configuration
==================================================================================================*/
#include <stdio.h>
#include "shell.h"
#include "FunctionLib.h"
#include "coap.h"
#include "fsl_os_abstraction.h"

#define USR_PB1_FLAG	0x01
#define USR_PB2_FLAG	0x02

typedef enum
{
	ledOff,
	ledRed,
	ledGreen,
	ledBlue,
	ledCyan,
	ledMagenta,
	ledYellow,
	ledWhite
}eLEDColor;

uint8_t gpio_expander_init(void);
uint8_t gpio_expander_get_keys(void);
uint8_t gpio_expander_set_led_color(eLEDColor color);
uint8_t gpio_expander_add_led_color(eLEDColor color);

#endif //_APP_GPIO_EXPANDER_H

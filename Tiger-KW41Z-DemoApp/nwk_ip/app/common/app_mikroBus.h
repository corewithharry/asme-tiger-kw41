#ifndef _APP_MIKROBUS_H             
#define _APP_MIKROBUS_H

/*!=================================================================================================
\file       app_mikroBus.h
\brief      This is the header file for mikroBus configuration
==================================================================================================*/
   
/*==================================================================================================
Include Files
==================================================================================================*/
#include "app_Relay.h"

#define MIKROBUS_ENABLE 1

#if MIKROBUS_ENABLE
  #if RELAY_CLICK_ENABLE
    #include "app_Relay.h"
  #endif
  
  #if ROTARY_ENABLE
  #endif

#endif

#endif
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

/*!=================================================================================================
\file       router_eligible_device_app_ota_client.c
\brief      This is a public source file for the router eligible device ota client demo application.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
/* General Includes */
#include "EmbeddedTypes.h"
#include <string.h>
#include <stdio.h>

/* FSL Framework */
#include "shell.h"
#include "Keyboard.h"
#include "RNG_Interface.h"

/* Network */
#include "ip_if_management.h"
#include "event_manager.h"

/* Application */
#include "router_eligible_device_app_ota_client.h"
#include "shell_ip.h"
#include "thread_utils.h"
#include "thread_meshcop.h"
#include "thread_network.h"
#include "thread_app_callbacks.h"
#include "app_init.h"
#include "app_stack_config.h"
#include "app_thread_config.h"
#include "app_led.h"
#include "app_temp_sensor.h"
#include "coap.h"
#include "app_socket_utils.h"
#include "app_accelerometer.h"
#if THR_ENABLE_EVENT_MONITORING
#include "app_event_monitoring.h"
#endif
#if THR_ENABLE_MGMT_DIAGNOSTICS
#include "thread_mgmt.h"
#include "thci.h"
#endif
#if UDP_ECHO_PROTOCOL
#include "app_echo_udp.h"
#endif
#if gEnableOTAClient_d
#include "app_ota.h"
#endif
#include "Eeprom.h"

#if VT_KW41Z_DEMO
    #include "app_registration.h"
    #if ACCELEROMETER_ENABLE
        #include "app_accelerometer.h"
    #endif /* ACCELEROMETER_ENABLE */
#endif /* VT_KW41Z_DEMO */
#if VT_KW41Z_MENP
    #if RGB_CLICK_ENABLE
        #include "app_RGBClick.h"
    #endif /* RGB_CLICK_ENABLE */
    #if RELAY_CLICK_ENABLE
        #include "app_Relay.h"
    #endif /* RELAY_CLICK_ENABLE */
#endif /* VT_KW41Z_MENP */
#include "app_pct2075_temp_sensor.h"
#include "app_FXAS21002_gyroscope.h"
#include "app_gpio_expander.h"
#include "app_FXLS8972CF_accelerometer.h"

#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"

#include "fsl_i2c.h"
#include "pin_mux.h"
#include "fsl_gpio.h"
#include "fsl_port.h"

/*==================================================================================================
Private macros
==================================================================================================*/
#ifndef APP_MSG_QUEUE_SIZE
    #define APP_MSG_QUEUE_SIZE                  20
#endif

#if (THREAD_USE_SHELL == FALSE)
    #define shell_write(a)
    #define shell_refresh()
    #define shell_printf(a,...)
#endif

#define gThrDefaultInstanceId_c                 0
#if APP_AUTOSTART
#define gAppFactoryResetTimeoutMin_c            10000
#define gAppFactoryResetTimeoutMax_c            20000
#endif
#define gAppRestoreLeaderLedTimeout_c           60     /* seconds */

#define gAppJoinTimeout_c                       800    /* miliseconds */

#define APP_LED_URI_PATH                        "/led"
#define APP_TEMP_URI_PATH                       "/temp"
#define APP_SINK_URI_PATH                       "/sink"
#if LARGE_NETWORK
#define APP_RESET_TO_FACTORY_URI_PATH           "/reset"
#endif
#if VT_KW41Z_DEMO
#define APP_KEY_EVENT_URI_PATH                  "/keyevent"
#define APP_FACTORY_RESET                       "/reset"
#if RELAY_CLICK_ENABLE
#define APP_RELAY_URI_PATH                       "/relay"
#endif /* RELAY_CLICK_ENABLE */
#endif /* VT_KW41Z_DEMO */

#define APP_DEFAULT_DEST_ADDR                   in6addr_realmlocal_allthreadnodes

#define APP_CREATE_NWK_IF_JOIN_FAILED           TRUE
#define EXTERNAL_MEMORY_INFORMATION_SIZE        5
#define THR_SW_VERSION_MAX_LEN                  10
#define MAJOR_FW_VERSION_BYTE 3
#define MINOR_FW_VERSION_BYTE 2

//Firmware version storage
union fwVersion {
    uint32_t versionValue;
    uint8_t  versionByte[4];
}currentVersion;

/*==================================================================================================
Private type definitions
==================================================================================================*/

/*==================================================================================================
Private global variables declarations
==================================================================================================*/
static instanceId_t mThrInstanceId = gInvalidInstanceId_c;    /*!< Thread Instance ID */

static bool_t mFirstPushButtonPressed = FALSE;

static bool_t mJoiningIsAppInitiated = FALSE;

static bool m_bUserLedUpdate;
static uint8_t m_redValue;
static uint8_t m_greenValue;
static uint8_t m_blueValue;

char eui_string[17];	//Zero terminated

#if (AT_COMMAND_SHELL)
static eLED_KEY_USE m_cLedKeyboardUse = atkAUTO;
#endif //AT_COMMAND_SHELL

/*==================================================================================================
Private prototypes
==================================================================================================*/
static void App_HandleKeyboard(void *param);
static void App_UpdateStateLeds(appDeviceState_t deviceState);
static void APP_JoinEventsHandler(thrEvCode_t evCode);
static void APP_InitCoapDemo(void);
static void APP_ReportTemp(void *pParam);
#ifndef FRDM_KW41Z
static void APP_SendDataSinkCreate(void *pParam);
static void APP_SendDataSinkRelease(void *pParam);
#endif
#if !(VT_KW41Z_MENP)
static void APP_SendLedRgbOn(void *pParam);
static void APP_TestCoapMsg(void *pParam);
static void APP_SendLedRgbOff(void *pParam);
static void APP_SendLedFlash(void *pParam);
static void APP_SendLedColorWheel(void *pParam);
#endif /* VT_KW41Z_DEMO */
static void APP_LocalDataSinkRelease(void *pParam);
static void APP_ProcessLedCmd(uint8_t *pCommand, uint8_t dataLen);
static void APP_CoapGenericCallback(coapSessionStatus_t sessionStatus, void *pData, coapSession_t *pSession, uint32_t dataLen);
static void APP_CoapLedCb(coapSessionStatus_t sessionStatus, void *pData, coapSession_t *pSession, uint32_t dataLen);
static void APP_CoapTempCb(coapSessionStatus_t sessionStatus, void *pData, coapSession_t *pSession, uint32_t dataLen);
static void APP_CoapSinkCb(coapSessionStatus_t sessionStatus, void *pData, coapSession_t *pSession, uint32_t dataLen);
#if VT_KW41Z_DEMO
static void APP_Reportkey_event(void *pParam);
static void APP_ReportSensors(char* pSensorString);
static void APP_CoapkeyeventCb(coapSessionStatus_t sessionStatus, void *pData, coapSession_t *pSession, uint32_t dataLen);
static void APP_CoapresetCb(coapSessionStatus_t sessionStatus, void *pData, coapSession_t *pSession, uint32_t dataLen);
#endif /* VT_KW41Z_DEMO */
static void App_RestoreLeaderLed(void * param);
#if LARGE_NETWORK
static void APP_CoapResetToFactoryDefaultsCb(coapSessionStatus_t sessionStatus, void *pData, coapSession_t *pSession, uint32_t dataLen);
static void APP_SendResetToFactoryCommand(void *param);
#endif
#if APP_AUTOSTART
static void APP_AutoStart(void *param);
static void APP_AutoStartCb(void *param);
#endif

/*==================================================================================================
Public global variables declarations
==================================================================================================*/
const coapUriPath_t gAPP_LED_URI_PATH  = {SizeOfString(APP_LED_URI_PATH), (uint8_t *)APP_LED_URI_PATH};
const coapUriPath_t gAPP_TEMP_URI_PATH = {SizeOfString(APP_TEMP_URI_PATH), (uint8_t *)APP_TEMP_URI_PATH};
const coapUriPath_t gAPP_SINK_URI_PATH = {SizeOfString(APP_SINK_URI_PATH), (uint8_t *)APP_SINK_URI_PATH};

#if VT_KW41Z_DEMO
const coapUriPath_t gAPP_KEY_EVENT_URI_PATH = {SizeOfString(APP_KEY_EVENT_URI_PATH), APP_KEY_EVENT_URI_PATH};
const coapUriPath_t gRESET_URI_PATH  = {SizeOfString(APP_FACTORY_RESET), APP_FACTORY_RESET};
#if ACCELEROMETER_ENABLE
const coapUriPath_t gACCELEROMETER_URI_PATH  = {SizeOfString(ACCELEROMETER_URI_PATH), ACCELEROMETER_URI_PATH};
#endif
#endif /* VT_KW41Z_DEMO */

#if VT_KW41Z_MENP
#if RGB_CLICK_ENABLE
const coapUriPath_t gRGBCLICK_URI_PATH  = {SizeOfString(RGBCLICK_URI_PATH), RGBCLICK_URI_PATH};
#endif
#if RELAY_CLICK_ENABLE
const coapUriPath_t gRELAY_URI_PATH  = {SizeOfString(APP_RELAY_URI_PATH), APP_RELAY_URI_PATH};
#endif /* RELAY_CLICK_ENABLE */
#endif /* VT_KW41Z_MENP */

#if LARGE_NETWORK
const coapUriPath_t gAPP_RESET_URI_PATH = {SizeOfString(APP_RESET_TO_FACTORY_URI_PATH), (uint8_t *)APP_RESET_TO_FACTORY_URI_PATH};
#endif

/* Application state/mode */
appDeviceState_t gAppDeviceState[THR_MAX_INSTANCES];
appDeviceMode_t gAppDeviceMode[THR_MAX_INSTANCES];

/* Flag used to stop the attaching retries */
bool_t gbRetryInterrupt = TRUE;

bool_t gbCreateNetwork = FALSE;

/* CoAP instance */
uint8_t mAppCoapInstId = THR_ALL_FFs8;
/* Destination address for CoAP commands */
ipAddr_t gCoapDestAddress;

/* Application timer Id */
tmrTimerID_t mAppTimerId = gTmrInvalidTimerID_c;

#if APP_AUTOSTART
tmrTimerID_t tmrStartApp = gTmrInvalidTimerID_c;
#endif

#if VT_KW41Z_DEMO
  static int key_event_count = 0;
#endif /* VT_KW41Z_DEMO */
  
#if VT_KW41Z_MENP
  bool_t gIsOTASupported = FALSE;
#endif /* VT_KW41Z_DEMO */

uint32_t leaderLedTimestamp = 0;

/* Pointer application task message queue */
taskMsgQueue_t *mpAppThreadMsgQueue = NULL;

extern bool_t gEnable802154TxLed;

/*==================================================================================================
Public functions
==================================================================================================*/
#if (!AT_COMMAND_SHELL)

//#define TEST_EEPROM
//#define INIT_NTAG
//#define TEST_SENSORS

#define SENSOR_POLL_INTERVAL_MS		1000
#define KEYS_POLL_INTERVAL_MS		100

#define ACCELEROMETER_FXOS8700

#define ARBITER_I2C_ADDR	0x70

#define ARB_CTRL_REG	0x01
	#define LOCK_REQ	0x01
	#define LOCK_GRANT	0x02
	#define BUS_CONNECT	0x04

unsigned char NFC_INIT_DATA[] =
{
	0xAA, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0xE1, 0x10, 0x10, 0x00
};

unsigned char NDEF_DATA[] =
{
	//NDEF URL Record
	0x03, 0x11, //NDEF message, 15 byte message
	0xD1,       // NDEF Record header: MB = 1b, ME = 1b, CF = 0b, SR = 1b, IL = 0b, TNF - 001b
	0x01, 0x0D, //type length, payload length
	0x55,       //Type = URI
	0x01,       //URI Indentifier: 1=http://www - 2=https://www - 3=http:// - 4=https://

	//Payload data = "iptronix.com"
	 'i', 'p', 't', 'r', 'o', 'n', 'i', 'x', '.', 'c', 'o', 'm',
	 0xFE,0,0,0,0,0,0,0,0,0,0,0,0
};

uint8_t ntag_init(void)
{
	uint8_t data_page[16];
    i2c_master_transfer_t masterXfer;
    uint8_t ndef_msg_blocks = (sizeof(NDEF_DATA) + 15)/16;
    int i;

    memset(&masterXfer, 0, sizeof(masterXfer));
    memset(data_page, 0, 16);

	masterXfer.slaveAddress = 0x55;
	masterXfer.direction = kI2C_Write;
	masterXfer.subaddress = 0;
	masterXfer.subaddressSize = 1;
	masterXfer.data = NFC_INIT_DATA;
	masterXfer.dataSize = 16;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	OSA_TimeDelay(100);
	if(I2C_MasterTransferBlocking(I2C0, &masterXfer) != kStatus_Success)
	{
		return 0;
	}

    for (i=1; i<=ndef_msg_blocks; i++)
    {
		masterXfer.slaveAddress = 0x55;
		masterXfer.direction = kI2C_Write;
		masterXfer.subaddress = i;
		masterXfer.subaddressSize = 1;
		masterXfer.data = data_page;
		masterXfer.dataSize = 16;
		masterXfer.flags = kI2C_TransferDefaultFlag;

		OSA_TimeDelay(100);
		if(I2C_MasterTransferBlocking(I2C0, &masterXfer) != kStatus_Success)
		{
			return 0;
		}
    }

    for (i=1; i<=ndef_msg_blocks; i++)
    {
		masterXfer.slaveAddress = 0x55;
		masterXfer.direction = kI2C_Write;
		masterXfer.subaddress = i;
		masterXfer.subaddressSize = 1;
		masterXfer.data = &NDEF_DATA[16*(i-1)];
		masterXfer.dataSize = 16;
		masterXfer.flags = kI2C_TransferDefaultFlag;

		OSA_TimeDelay(100);
		if(I2C_MasterTransferBlocking(I2C0, &masterXfer) != kStatus_Success)
		{
			return 0;
		}
    }

    data_page[0] = 0xFE;
    data_page[1] = 0x01;
    data_page[2] = 0xFF;
    data_page[3] = 0x02;

	masterXfer.slaveAddress = 0x55;
	masterXfer.direction = kI2C_Write;
	masterXfer.subaddress = 0;
	masterXfer.subaddressSize = 0;
	masterXfer.data = data_page;
	masterXfer.dataSize = 4;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	OSA_TimeDelay(100);
	if(I2C_MasterTransferBlocking(I2C0, &masterXfer) != kStatus_Success)
	{
		return 0;
	}

    data_page[0] = 0x3A;
    data_page[1] = 0x01;
    data_page[2] = 0xFF;
    data_page[3] = 0x02;

	masterXfer.slaveAddress = 0x55;
	masterXfer.direction = kI2C_Write;
	masterXfer.subaddress = 0;
	masterXfer.subaddressSize = 0;
	masterXfer.data = data_page;
	masterXfer.dataSize = 4;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	OSA_TimeDelay(100);
	if(I2C_MasterTransferBlocking(I2C0, &masterXfer) != kStatus_Success)
	{
		return 0;
	}

	return 1;
}

uint8_t arbiter_write_register(uint8_t reg, uint8_t data)
{
    i2c_master_transfer_t masterXfer;

    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress = ARBITER_I2C_ADDR;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = reg;
    masterXfer.subaddressSize = 1;
    masterXfer.data = &data;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    if(I2C_MasterTransferBlocking(I2C0, &masterXfer) == kStatus_Success)
    {
    	return 1;
    }

    return 0;
}

uint8_t arbiter_read_register(uint8_t reg, uint8_t* pcData)
{
    i2c_master_transfer_t masterXfer;

    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress = ARBITER_I2C_ADDR;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = reg;
    masterXfer.subaddressSize = 1;
    masterXfer.data = pcData;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    if(I2C_MasterTransferBlocking(I2C0, &masterXfer) == kStatus_Success)
    {
    	return 1;
    }

    return 0;
}

uint8_t arbiter_get_downstream_bus(void)
{
	uint8_t ctrl_reg;

	if (!arbiter_read_register(ARB_CTRL_REG, &ctrl_reg))
	{
		return 0;
	}

	//if already connected return 1
	if ((ctrl_reg & (BUS_CONNECT | LOCK_GRANT | LOCK_REQ)) == (BUS_CONNECT | LOCK_GRANT | LOCK_REQ))
	{
		return 1;
	}

	ctrl_reg |= LOCK_REQ;
	if (!arbiter_write_register(ARB_CTRL_REG, ctrl_reg))
	{
		return 0;
	}

	do
	{
		if (!arbiter_read_register(ARB_CTRL_REG, &ctrl_reg))
		{
			return 0;
		}
	}while(!(ctrl_reg & LOCK_GRANT));

	ctrl_reg |= BUS_CONNECT;
	if (!arbiter_write_register(ARB_CTRL_REG, ctrl_reg))
	{
		return 0;
	}

	return 1;
}

uint8_t arbiter_release_downstream_bus(void)
{
	uint8_t ctrl_reg;

	if (!arbiter_read_register(ARB_CTRL_REG, &ctrl_reg))
	{
		return 0;
	}

	ctrl_reg &= ~(LOCK_REQ | BUS_CONNECT);
	if (!arbiter_write_register(ARB_CTRL_REG, ctrl_reg))
	{
		return 0;
	}

	return 1;
}

void SensorBoardInit(void)
{
    i2c_master_config_t masterConfig;
	uint8_t* peui;
    uint32_t eui_size;
    uint64_t eui_rev;

	THR_GetAttr(mThrInstanceId, gNwkAttrId_IeeeAddr_c, 0, &eui_size, &eui_rev);
	NWKU_SwapArrayBytes((uint8_t *)&eui_rev, 8);
	peui = (uint8_t *)&eui_rev;
	for (int i=0; i<eui_size; i++)
	{
		sprintf(&eui_string[2*i], "%02X", *peui);
		peui++;
	}

	BOARD_InitI2C();
    I2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Bps = 100000U;
    I2C_MasterInit(I2C0, &masterConfig, BOARD_GetI2cClock(0));

    if (arbiter_get_downstream_bus())
    {
    	gpio_expander_init();

#ifdef ACCELEROMETER_FXOS8700
		accelerometerInit();
#else //ACCELEROMETER_FXOS8700
		accelerometer_FXLS8972_Init();
#endif //ACCELEROMETER_FXOS8700
    	gyroscope_init();

#ifdef INIT_NTAG
    	ntag_init();
#endif //INIT_NTAG

        arbiter_release_downstream_bus();
    }

    m_bUserLedUpdate = false;
}

void SensorBoardHandle(void)
{
	static TickType_t last_sensors_ticks = 0;
	static TickType_t last_keys_ticks = 0;
	static uint8_t prev_key_status = 0;
	TickType_t now;
    char sensorsDataStr[256];
    uint16_t sensor_strlen;

#ifndef TEST_SENSORS
    if(APP_GetMode(mThrInstanceId) == gDeviceMode_Configuration_c)
    {
    	return;
    }
#endif //TEST_SENSORS

    now = xTaskGetTickCount();

    if ((now - last_sensors_ticks) >= SENSOR_POLL_INTERVAL_MS ||
    	(now - last_keys_ticks) >= KEYS_POLL_INTERVAL_MS ||
		m_bUserLedUpdate)
    {
		if (arbiter_get_downstream_bus())
		{
			if ((now - last_sensors_ticks) >= SENSOR_POLL_INTERVAL_MS)
			{
				sensorsDataStr[0] = '{';

				//Thread get eui
            	sprintf(&sensorsDataStr[1], "\"eui\":\"%s\",", eui_string);
				sensor_strlen = strlen(sensorsDataStr);
				//Read accelerometer
#ifdef ACCELEROMETER_FXOS8700
				App_GetAccDataString(&sensorsDataStr[sensor_strlen]);
#else //ACCELEROMETER_FXOS8700
				accelerometer_FXLS8972_get_data_string(&sensorsDataStr[sensor_strlen]);
#endif //ACCELEROMETER_FXOS8700
				sensor_strlen = strlen(sensorsDataStr);
				sensorsDataStr[sensor_strlen++] = ',';

				gyroscope_get_data_string(&sensorsDataStr[sensor_strlen]);
				sensor_strlen = strlen(sensorsDataStr);
				sensorsDataStr[sensor_strlen++] = ',';

				pct2075_get_temperature_string(&sensorsDataStr[sensor_strlen]);
				sensor_strlen = strlen(sensorsDataStr);
				sensorsDataStr[sensor_strlen++] = '}';
				sensorsDataStr[sensor_strlen] = 0;

				last_sensors_ticks = now;
#ifndef TEST_SENSORS
				APP_ReportSensors(sensorsDataStr);
#else //TEST_SENSORS
				//shell_printf(sensorsDataStr);
				shell_writeN(sensorsDataStr, sensor_strlen);
				shell_writeN("\n\r", 2);
#endif //TEST_SENSORS
			}

			if ((now - last_keys_ticks) >= KEYS_POLL_INTERVAL_MS)
			{
				uint8_t key_status;
				key_status = gpio_expander_get_keys();
				last_keys_ticks = now;
				if ((key_status & USR_PB1_FLAG) && prev_key_status == 0)
				{
					key_event_count++;
#ifndef TEST_SENSORS
					APP_Reportkey_event(NULL);
#else //TEST_SENSORS
					m_bUserLedUpdate = true;
#endif //TEST_SENSORS
				}
				else if ((key_status & USR_PB2_FLAG) && prev_key_status == 0)
				{
					key_event_count--;
#ifndef TEST_SENSORS
					APP_Reportkey_event(NULL);
#else //TEST_SENSORS
					m_bUserLedUpdate = true;
#endif //TEST_SENSORS
				}

				prev_key_status = key_status;
			}

			if (m_bUserLedUpdate)
			{
				m_bUserLedUpdate = false;
#ifdef TEST_SENSORS
				m_redValue = (key_event_count & 0x01)? 1 : 0;
				m_greenValue = (key_event_count & 0x02)? 1 : 0;
				m_blueValue = (key_event_count & 0x04)? 1 : 0;
#endif //TEST_SENSORS
		        /* Update RGB values */
		        gpio_expander_set_led_color(ledOff);
		        if (m_redValue)
		        {
		            gpio_expander_add_led_color(ledRed);
		        }
		        if (m_greenValue)
		        {
		            gpio_expander_add_led_color(ledGreen);
		        }
		        if (m_blueValue)
		        {
		            gpio_expander_add_led_color(ledBlue);
		        }

			}

			arbiter_release_downstream_bus();
		}
    }
}
#else //AT_COMMAND_SHELL
void APP_LedKeyboardStatusSet(eLED_KEY_USE use)
{
	m_cLedKeyboardUse = use;

	if (m_cLedKeyboardUse != atkAUTO)
	{
		LedSetATConfig(0, 0, 0);
	}
	else
	{
		App_UpdateStateLeds(APP_GetState(mThrInstanceId));
	}
}

eLED_KEY_USE APP_LedKeyboardStatusGet(void)
{
	return m_cLedKeyboardUse;
}

#endif //AT_COMMAND_SHELL

/*!*************************************************************************************************
\fn     void APP_Init(void)
\brief  This function is used to initialize application.
***************************************************************************************************/
void APP_Init
(
    void
)
{
#if VT_KW41Z_MENP
    uint8_t retStatus = ee_ok;
    uint8_t extMemInfo[EXTERNAL_MEMORY_INFORMATION_SIZE];
    uint8_t AT45DB081E_MemInfo[EXTERNAL_MEMORY_INFORMATION_SIZE] = {0x1F,0x25,0x00,0x01,0x00};
#endif /* VT_KW41Z_MENP */

#ifdef TEST_EEPROM
    uint8_t eeprom_test_data[100];
#endif //TEST_EEPROM

    /* Initialize pointer to application task message queue */
    mpAppThreadMsgQueue = &appThreadMsgQueue;

    /* Initialize main thread message queue */
    ListInit(&appThreadMsgQueue.msgQueue,APP_MSG_QUEUE_SIZE);

    /* Set default device mode/state */
    APP_SetState(gThrDefaultInstanceId_c, gDeviceState_FactoryDefault_c);
    APP_SetMode(gThrDefaultInstanceId_c, gDeviceMode_Configuration_c);

    /* Initialize keyboard handler */
    pfAppKeyboardHandler = App_HandleKeyboard;

    /* Use one instance ID for application */
    mThrInstanceId = gThrDefaultInstanceId_c;

#if THR_ENABLE_EVENT_MONITORING
    /* Initialize event monitoring */
    APP_InitEventMonitor(mThrInstanceId);
#endif

#ifdef TEST_EEPROM
    EEPROM_Init();
    if (EEPROM_ReadData(100, 0x1234, eeprom_test_data) == ee_ok)
    {
    	for (int i=0; i<100; i++)
    	{
    		eeprom_test_data[i] = 99-i;
    	}

    	if (EEPROM_WriteData(100, 0x1234, eeprom_test_data) == ee_ok)
    	{
        	for (int i=0; i<100; i++)
        	{
        		eeprom_test_data[i] = 0;
        	}

        	EEPROM_ReadData(100, 0x1234, eeprom_test_data);
    	}
    }

#endif //TEST_EEPROM

#if VT_KW41Z_MENP
    #if RGB_CLICK_ENABLE
       RGBLed_Init();
    #endif /* RGB_CLICK_ENABLE */
    #if RELAY_CLICK_ENABLE
            Relay_Init();
            Led1On();
    #endif /* RELAY_CLICK_ENABLE */
#endif /* VT_KW41Z_MENP */

    if(gThrStatus_Success_c == THR_StartInstance(mThrInstanceId, pStackCfg[0]))
    {
        /* Initialize CoAP demo */
        APP_InitCoapDemo();

#if USE_TEMPERATURE_SENSOR
        /* Initialize Temperature sensor/ADC module*/
        APP_InitADC(ADC_0);
#endif

#if THREAD_USE_THCI && THR_ENABLE_MGMT_DIAGNOSTICS
        (void)MgmtDiagnostic_RegisterAppCb(THCI_MgmtDiagnosticAppCb);
#endif

#if THREAD_USE_SHELL && SOCK_DEMO
        /* Initialize use sockets - used from shell */
        APP_InitUserSockets(mpAppThreadMsgQueue);
#endif
     
     /* Display current firmware version */
     currentVersion.versionValue = gOtaCurrentFileVersionNo_c;
#if (!AT_COMMAND_SHELL)
     shell_printf("Thread end node firmware version: %x.%x.%x\r\n", (currentVersion.versionByte[MAJOR_FW_VERSION_BYTE] >> 4 & 0x0F),
                                                                    (currentVersion.versionByte[MAJOR_FW_VERSION_BYTE] & 0x0F),
                                                                    (currentVersion.versionByte[MINOR_FW_VERSION_BYTE]));
#endif //AT_COMMAND_SHELL
#if VT_KW41Z_MENP
    /* Check for the External memory availability */
    retStatus = EEPROM_Init();
    if(retStatus != ee_ok)
    {
      shell_printf("Failed to init the external memory\r\n");
      shell_printf("Disabling the OTA support\r\n");
      gIsOTASupported = FALSE;
    }
    else
    {
      /* Now read External memory Id for the verification */
      memset(extMemInfo,0x00,EXTERNAL_MEMORY_INFORMATION_SIZE);
      EEPROM_ReadBasicInfo(extMemInfo);
      if(memcmp(extMemInfo,AT45DB081E_MemInfo,EXTERNAL_MEMORY_INFORMATION_SIZE) == 0)
      {
        shell_printf("Thread end node supports OTA\r\n");
        gIsOTASupported = TRUE;
      }
      else
      {
        shell_printf("Thread end node does not supports OTA\r\n");
        gIsOTASupported = FALSE;
      }
    }

if(gIsOTASupported && gEnableOTAClient_d)
{
  OtaClientInit(mThrInstanceId, mpAppThreadMsgQueue);
}
#else
#if gEnableOTAClient_d
        OtaClientInit(mThrInstanceId, mpAppThreadMsgQueue);
#endif /* gEnableOTAClient_d */
#endif /* VT_KW41Z_MENP */

#if NTAG_I2C && VT_KW41Z_MENP
  InitNTAG();   //init I2C NTAG
  WriteTag();        
#endif

#if APP_AUTOSTART
        tmrStartApp = TMR_AllocateTimer();

        if(tmrStartApp != gTmrInvalidTimerID_c)
        {
            uint32_t jitterTime = NWKU_GetRandomNoFromInterval(gAppFactoryResetTimeoutMin_c,
                                                               gAppFactoryResetTimeoutMax_c);
            TMR_StartSingleShotTimer(tmrStartApp, jitterTime, APP_AutoStartCb, NULL);
        }
#endif

    }

#if (!AT_COMMAND_SHELL)
    SensorBoardInit();
#endif //AT_COMMAND_SHELL
}

/*!*************************************************************************************************
\fn     void App_Handler(void)
\brief  Application Handler. In this configuration is called on the task with the lowest priority
***************************************************************************************************/
void APP_Handler
(
    void
)
{
    bool_t handleMsg = TRUE;

#if (!AT_COMMAND_SHELL)
    SensorBoardHandle();
#endif //AT_COMMAND_SHELL

    while(handleMsg == TRUE)
    {
        handleMsg = NWKU_MsgHandler(&appThreadMsgQueue);
        /* For BareMetal break the while(1) after 1 run */
        if(!gUseRtos_c && MSG_Pending(&appThreadMsgQueue.msgQueue))
        {
            (void)OSA_EventSet(appThreadMsgQueue.taskEventId, NWKU_GENERIC_MSG_EVENT);
            break;
        }

    }
}

/*!*************************************************************************************************
\fn     void APP_NwkScanHandler(void *param)
\brief  This function is used to handle network scan results in asynchronous mode.

\param  [in]    param    Pointer to stack event
***************************************************************************************************/
void APP_NwkScanHandler
(
    void *param
)
{
    thrEvmParams_t *pEventParams = (thrEvmParams_t *)param;
    thrNwkScanResults_t *pScanResults = &pEventParams->pEventData->nwkScanCnf;

    /* Handle the network scan result here */
    if(pScanResults)
    {
#if THREAD_USE_SHELL
        SHELL_NwkScanPrint(pScanResults);
#endif
        MEM_BufferFree(pScanResults);
    }
    /* Free Event Buffer */
    MEM_BufferFree(pEventParams);
}

/*!*************************************************************************************************
\fn     void Stack_to_APP_Handler(void *param)
\brief  This function is used to handle stack events in asynchronous mode.

\param  [in]    param    Pointer to stack event
***************************************************************************************************/
void Stack_to_APP_Handler
(
    void *param
)
{
    thrEvmParams_t *pEventParams = (thrEvmParams_t *)param;

    switch(pEventParams->code)
    {
        case gThrEv_GeneralInd_ResetToFactoryDefault_c:
            App_UpdateStateLeds(gDeviceState_FactoryDefault_c);
            break;

        case gThrEv_GeneralInd_InstanceRestoreStarted_c:
        case gThrEv_GeneralInd_ConnectingStarted_c:
            APP_SetMode(mThrInstanceId, gDeviceMode_Configuration_c);
            App_UpdateStateLeds(gDeviceState_JoiningOrAttaching_c);
            gEnable802154TxLed = FALSE;
            break;

        case gThrEv_NwkJoinCnf_Success_c:
        case gThrEv_NwkJoinCnf_Failed_c:
            APP_JoinEventsHandler(pEventParams->code);
            break;

        case gThrEv_GeneralInd_Connected_c:
            App_UpdateStateLeds(gDeviceState_NwkConnected_c);
            /* Set application CoAP destination to all nodes on connected network */
            gCoapDestAddress = APP_DEFAULT_DEST_ADDR;
            APP_SetMode(mThrInstanceId, gDeviceMode_Application_c);
            mFirstPushButtonPressed  = FALSE;
            /* Synchronize server data */
            THR_BrPrefixAttrSync(mThrInstanceId);
            /* Enable LED for 80215.4 tx activity */
#ifndef VT_KW41Z_MENP
            gEnable802154TxLed = TRUE;
            #if gEnableOTAClient_d
            //(void)NWKU_SendMsg(OtaClient_StartServerDiscovery, NULL, mpAppThreadMsgQueue);
            #endif 
#endif /* VT_KW41Z_MENP */

#ifdef VT_KW41Z_DEMO
            App_Task_Init();
#endif /* VT_KW41Z_DEMO */
            break;

        case gThrEv_GeneralInd_RequestRouterId_c:
            gEnable802154TxLed = FALSE;
            break;

        case gThrEv_GeneralInd_ConnectingDeffered_c:
            APP_SetMode(mThrInstanceId, gDeviceMode_Configuration_c);
            gEnable802154TxLed = FALSE;
            App_UpdateStateLeds(gDeviceState_NwkOperationPending_c);
            break;

        case gThrEv_GeneralInd_ConnectingFailed_c:
        case gThrEv_GeneralInd_Disconnected_c:
            APP_SetMode(mThrInstanceId, gDeviceMode_Configuration_c);
            App_UpdateStateLeds(gDeviceState_NwkFailure_c);
            break;

        case gThrEv_GeneralInd_DeviceIsLeader_c:
            App_UpdateStateLeds(gDeviceState_Leader_c);
#ifndef VT_KW41Z_MENP
            gEnable802154TxLed = TRUE;
#endif /* VT_KW41Z_MENP */
#if !LARGE_NETWORK
            /* Auto start commissioner for the partition for demo purposes */
            MESHCOP_StartCommissioner(pEventParams->thrInstId);
#endif
            break;

        case gThrEv_GeneralInd_DeviceIsRouter_c:
            App_UpdateStateLeds(gDeviceState_ActiveRouter_c);
#ifndef VT_KW41Z_MENP
            gEnable802154TxLed = TRUE;
#endif /* VT_KW41Z_MENP */
#if UDP_ECHO_PROTOCOL
            ECHO_ProtocolInit(mpAppThreadMsgQueue);
#endif
            break;

        case gThrEv_GeneralInd_DevIsREED_c:
            App_UpdateStateLeds(gDeviceState_NwkConnected_c);
#ifndef VT_KW41Z_MENP
            gEnable802154TxLed = TRUE;
#endif /* VT_KW41Z_MENP */
            break;

#if gLpmIncluded_d
        case gThrEv_GeneralInd_AllowDeviceToSleep_c:
            PWR_AllowDeviceToSleep();
            break;

        case gThrEv_GeneralInd_DisallowDeviceToSleep_c:
            PWR_DisallowDeviceToSleep();
            break;
#endif
        default:
            break;
    }

    /* Free event buffer */
    MEM_BufferFree(pEventParams->pEventData);
    MEM_BufferFree(pEventParams);
}

/*!*************************************************************************************************
\fn     void APP_Commissioning_Handler(void *param)
\brief  This function is used to handle Commissioning events in synchronous mode.

\param  [in]    param    Pointer to Commissioning event
***************************************************************************************************/
void APP_Commissioning_Handler
(
    void *param
)
{
    thrEvmParams_t *pEventParams = (thrEvmParams_t *)param;

    switch(pEventParams->code)
    {
        /* Joiner Events */
        case gThrEv_MeshCop_JoinerDiscoveryStarted_c:
            break;
        case gThrEv_MeshCop_JoinerDiscoveryFailed_c:
            break;
        case gThrEv_MeshCop_JoinerDiscoveryFailedFiltered_c:
            break;
        case gThrEv_MeshCop_JoinerDiscoverySuccess_c:
            break;
        case gThrEv_MeshCop_JoinerDtlsSessionStarted_c:
            App_UpdateStateLeds(gDeviceState_JoiningOrAttaching_c);
            break;
        case gThrEv_MeshCop_JoinerDtlsError_c:
        case gThrEv_MeshCop_JoinerError_c:
            App_UpdateStateLeds(gDeviceState_FactoryDefault_c);
            break;
        case gThrEv_MeshCop_JoinerAccepted_c:
            break;

        /* Commissioner Events(event set applies for all Commissioners: on-mesh, external, native) */
        case gThrEv_MeshCop_CommissionerPetitionStarted_c:
            break;
        case gThrEv_MeshCop_CommissionerPetitionAccepted_c:
        {
            uint8_t aDefaultEui[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
            thrOctet32_t defaultPskD = THR_PSK_D;

            MESHCOP_AddExpectedJoiner(mThrInstanceId, aDefaultEui, defaultPskD.aStr, defaultPskD.length, TRUE);
            MESHCOP_SyncSteeringData(mThrInstanceId, gMeshcopEuiMaskAllFFs_c);
            break;
        }
        case gThrEv_MeshCop_CommissionerPetitionRejected_c:
            break;
        case gThrEv_MeshCop_CommissionerPetitionError_c:
            break;
        case gThrEv_MeshCop_CommissionerKeepAliveSent_c:
            break;
        case gThrEv_MeshCop_CommissionerError_c:
            break;
        case gThrEv_MeshCop_CommissionerJoinerDtlsSessionStarted_c:
            break;
        case gThrEv_MeshCop_CommissionerJoinerDtlsError_c:
            break;
        case gThrEv_MeshCop_CommissionerJoinerAccepted_c:
            break;
        case gThrEv_MeshCop_CommissionerNwkDataSynced_c:
            break;
    }

    /* Free event buffer */
    MEM_BufferFree(pEventParams);
}

/*!*************************************************************************************************
\fn     void App_RestoreLeaderLedCb(void *param)
\brief  Called in Application state to restore leader LED.

\param  [in]    param    Not used
***************************************************************************************************/
void App_RestoreLeaderLedCb
(
    void *param
)
{
    (void)NWKU_SendMsg(App_RestoreLeaderLed, NULL, mpAppThreadMsgQueue);
}

/*==================================================================================================
Private functions
==================================================================================================*/
#if (AT_COMMAND_SHELL)
extern void SHELL_InitUserURI(void);
#endif //AT_COMMAND_SHELL

/*!*************************************************************************************************
\private
\fn     static void APP_InitCoapDemo(void)
\brief  Initialize CoAP demo.
***************************************************************************************************/
static void APP_InitCoapDemo
(
    void
)
{
    coapRegCbParams_t cbParams[] =  {{APP_CoapLedCb,  (coapUriPath_t *)&gAPP_LED_URI_PATH},
                                     {APP_CoapTempCb, (coapUriPath_t *)&gAPP_TEMP_URI_PATH},

#if VT_KW41Z_DEMO
                                    {APP_CoapkeyeventCb,(coapUriPath_t*)&gAPP_KEY_EVENT_URI_PATH},
                                    {APP_CoapresetCb,  (coapUriPath_t*)&gRESET_URI_PATH},
                                    {Accelerometer_CoapCb,  (coapUriPath_t*)&gACCELEROMETER_URI_PATH},
#endif /* VT_KW41Z_DEMO */

#if VT_KW41Z_MENP
#if RGB_CLICK_ENABLE                                    
                                    {APP_CoapRGBCb,  (coapUriPath_t*)&gRGBCLICK_URI_PATH},
#endif /* RGB_CLICK_ENABLE */
#if RELAY_CLICK_ENABLE                                    
                                    {APP_CoapRelayCb,  (coapUriPath_t*)&gRELAY_URI_PATH},
#endif /* RELAY_CLICK_ENABLE */
#endif /* VT_KW41Z_MENP */

#if LARGE_NETWORK
                                     {APP_CoapResetToFactoryDefaultsCb, (coapUriPath_t *)&gAPP_RESET_URI_PATH},
#endif
                                     {APP_CoapSinkCb, (coapUriPath_t *)&gAPP_SINK_URI_PATH}};
    /* Register Services in COAP */
    coapStartUnsecParams_t coapParams = {COAP_DEFAULT_PORT, AF_INET6};
    mAppCoapInstId = COAP_CreateInstance(NULL, &coapParams, gIpIfSlp0_c, (coapRegCbParams_t *)cbParams,
                                         NumberOfElements(cbParams));
#if (AT_COMMAND_SHELL)
    SHELL_InitUserURI();
#endif //AT_COMMAND_SHELL
}

/*!*************************************************************************************************
\private
\fn     static void APP_ThrNwkJoin(void *param)
\brief  Start the joining procedure.

\param  [in]    param    Not used
***************************************************************************************************/
static void APP_ThrNwkJoin
(
    void *param
)
{
    if(THR_NwkJoin(mThrInstanceId, THR_APP_JOIN_DISCOVERY_METHOD) != gThrStatus_Success_c)
    {
        /* User can treat join failure according to their application */
    }
}

/*!*************************************************************************************************
\private
\fn     static void App_JoinTimerCallback(void *param)
\brief  Join timer callback.

\param  [in]    param    Not used
***************************************************************************************************/
static void App_JoinTimerCallback
(
    void *param
)
{
    if(mFirstPushButtonPressed)
    {
        mJoiningIsAppInitiated = TRUE;
        TMR_FreeTimer(mAppTimerId);
        mAppTimerId = gTmrInvalidTimerID_c;
        (void)NWKU_SendMsg(APP_ThrNwkJoin, NULL, mpAppThreadMsgQueue);
    }
}

/*!*************************************************************************************************
\private
\fn     static void APP_ConfigModeSwShortPressHandler(uint32_t keyEvent)
\brief  This is a handler for  KBD module - short press events. Device is in configuration mode.

\param  [in]    keyEvent    The keyboard module event
***************************************************************************************************/
static void APP_ConfigModeSwShortPressHandler
(
    uint32_t keyEvent
)
{
    (void)keyEvent;

    if((APP_GetState(mThrInstanceId) == gDeviceState_FactoryDefault_c) ||
       (APP_GetState(mThrInstanceId) == gDeviceState_NwkFailure_c) ||
       (APP_GetState(mThrInstanceId) == gDeviceState_NwkOperationPending_c))
    {
        App_UpdateStateLeds(gDeviceState_JoiningOrAttaching_c);
        mFirstPushButtonPressed = TRUE;

        if(mAppTimerId == gTmrInvalidTimerID_c)
        {
            mAppTimerId = TMR_AllocateTimer();
        }

        /* Validate application timer Id */
        if(mAppTimerId != gTmrInvalidTimerID_c)
        {
            /* Start the application timer. Wait gAppJoinTimeout_c
               to start the joining procedure */
            TMR_StartSingleShotTimer(mAppTimerId, gAppJoinTimeout_c, App_JoinTimerCallback, NULL);
        }
        else
        {
            mJoiningIsAppInitiated = TRUE;
            /* No timer available - try to join the network */
            if(THR_NwkJoin(mThrInstanceId, THR_APP_JOIN_DISCOVERY_METHOD) != gThrStatus_Success_c)
            {
                /* User can treat join failure according to their application */
            }
        }
    }
    /* Double press */
    else if(mFirstPushButtonPressed)
    {
#ifndef VT_KW41Z_MENP
        /* Reset */
        mFirstPushButtonPressed = FALSE;

        if((mJoiningIsAppInitiated == FALSE) &&
           (!THR_GetAttr_IsDevConnected(mThrInstanceId)))
        {
            if(mAppTimerId != gTmrInvalidTimerID_c)
            {
                TMR_FreeTimer(mAppTimerId);
                mAppTimerId = gTmrInvalidTimerID_c;
            }

            App_UpdateStateLeds(gDeviceState_Leader_c);
            /* Create the network */
            (void)THR_NwkCreate(mThrInstanceId);
        }
        else
        {
            /* Create network */
            gbCreateNetwork = TRUE;
            /* Device will create the network after receiving the next gThrEv_NwkJoinCnf_Failed_c event */
        }
#endif /* VT_KW41Z_MENP */
    }
}

/*!*************************************************************************************************
\private
\fn     static void APP_ConfigModeHandleKeyboard(uint32_t keyEvent)
\brief  This is a handler for KBD module events. Device is in configuration mode.

\param  [in]    keyEvent   The keyboard module event
***************************************************************************************************/
static void APP_ConfigModeHandleKeyboard
(
    uint32_t keyEvent
)
{
    switch(keyEvent)
    {
        case gKBD_EventPB1_c:
#if gKBD_KeysCount_c > 1
        case gKBD_EventPB2_c:
        case gKBD_EventPB3_c:
        case gKBD_EventPB4_c:
#endif
            APP_ConfigModeSwShortPressHandler(keyEvent);
            break;
        case gKBD_EventLongPB1_c:
#if gKBD_KeysCount_c > 1
        case gKBD_EventLongPB2_c:
        case gKBD_EventLongPB3_c:
        case gKBD_EventLongPB4_c:
#endif
            break;
        case gKBD_EventVeryLongPB1_c:
#if gKBD_KeysCount_c > 1
        case gKBD_EventVeryLongPB2_c:
        case gKBD_EventVeryLongPB3_c:
        case gKBD_EventVeryLongPB4_c:
#endif
            /* Factory reset */
            THR_FactoryReset();
            break;
        default:
            break;
    }
}

/*!*************************************************************************************************
\private
\fn     static void APP_AppModeHandleKeyboard(uint32_t keyEvent)
\brief  This is a handler for KBD module events. Device is in application mode.

\param  [in]    keyEvent   The keyboard module event
***************************************************************************************************/
static void APP_AppModeHandleKeyboard
(
    uint32_t keyEvent
)
{
    switch(keyEvent)
    {
        case gKBD_EventPB1_c:
#if VT_KW41Z_MENP
             /* Report key event */
            (void)NWKU_SendMsg(APP_Reportkey_event, NULL, mpAppThreadMsgQueue);
#else
#ifdef FRDM_KW41Z
            /* Remote led RGB - on */
            //(void)NWKU_SendMsg(APP_SendLedRgbOn, NULL, mpAppThreadMsgQueue);
            (void)NWKU_SendMsg(APP_TestCoapMsg, NULL, mpAppThreadMsgQueue);
#else
            /* Data sink create */
            (void)NWKU_SendMsg(APP_SendDataSinkCreate, NULL, mpAppThreadMsgQueue);
#endif
#endif /* VT_KW41Z_MENP */ 
            break;
#if gKBD_KeysCount_c > 1
        case gKBD_EventPB2_c:
#if VT_KW41Z_DEMO
            (void)NWKU_SendMsg(APP_Reportkey_event, NULL, mpAppThreadMsgQueue);
#else
#ifdef FRDM_KW41Z
            /* Remote led RGB - off */
            (void)NWKU_SendMsg(APP_SendLedRgbOff, NULL, mpAppThreadMsgQueue);
#else
            /* Report temperature */
            (void)NWKU_SendMsg(APP_ReportTemp, NULL, mpAppThreadMsgQueue);
#endif
#endif /* VT_KW41Z_DEMO */ 
            break;
        case gKBD_EventPB3_c:
            /* Remote led RGB - on */
            (void)NWKU_SendMsg(APP_SendLedRgbOn, NULL, mpAppThreadMsgQueue);
            break;
        case gKBD_EventPB4_c:
            /* Remote led RGB - off */
            (void)NWKU_SendMsg(APP_SendLedRgbOff, NULL, mpAppThreadMsgQueue);
            break;
#endif
        case gKBD_EventLongPB1_c:
#ifdef FRDM_KW41Z
            /* Report temperature */
            (void)NWKU_SendMsg(APP_ReportTemp, NULL, mpAppThreadMsgQueue);
#else
            /* Remote data sink release */
            (void)NWKU_SendMsg(APP_SendDataSinkRelease, NULL, mpAppThreadMsgQueue);
#endif
            break;
#if gKBD_KeysCount_c > 1
        case gKBD_EventLongPB2_c:
#ifdef FRDM_KW41Z
            /* Remote led flash */
            (void)NWKU_SendMsg(APP_SendLedFlash, NULL, mpAppThreadMsgQueue);
            break;
#else
            /* Local data sink release */
            (void)NWKU_SendMsg(APP_LocalDataSinkRelease, NULL, mpAppThreadMsgQueue);
#endif
            break;
        case gKBD_EventLongPB3_c:
            /* Remote led flash */
            (void)NWKU_SendMsg(APP_SendLedFlash, NULL, mpAppThreadMsgQueue);
            break;
        case gKBD_EventLongPB4_c:
            /* Remote led - color wheel */
            (void)NWKU_SendMsg(APP_SendLedColorWheel, NULL, mpAppThreadMsgQueue);
            break;
#endif
        case gKBD_EventVeryLongPB1_c:
            /* Factory reset,  not for FRDM_KW24D512*/
#ifdef FRDM_KW24
            OtaClient_StartServerDiscovery(NULL);
            break;
#endif
#if gKBD_KeysCount_c > 1
        case gKBD_EventVeryLongPB4_c:
#if LARGE_NETWORK
            /* OTA factory reset */
            (void)NWKU_SendMsg(APP_SendResetToFactoryCommand, NULL, mpAppThreadMsgQueue);
            break;
#endif
        case gKBD_EventVeryLongPB3_c:
        case gKBD_EventVeryLongPB2_c:
#endif
            /* Factory reset */
            THR_FactoryReset();
            break;
        default:
            break;
    }
}

/*!*************************************************************************************************
\private
\fn     static void App_HandleKeyboard(void *param)
\brief  This is a handler for KBD module events.

\param  [in]    param    The keyboard module event
***************************************************************************************************/
static void App_HandleKeyboard
(
    void *param
)
{
    uint32_t events = (uint32_t)(param);

#if (AT_COMMAND_SHELL)
    if (m_cLedKeyboardUse == atkAUTO)
    {
#endif //AT_COMMAND_SHELL
		if(APP_GetMode(mThrInstanceId) == gDeviceMode_Configuration_c)
		{
			/* Device is in configuration mode */
			APP_ConfigModeHandleKeyboard(events);
		}
		else
		{
			/* Device is in application mode */
			APP_AppModeHandleKeyboard(events);
		}
#if (AT_COMMAND_SHELL)
    }
    else if (m_cLedKeyboardUse == atkAT_ASYNC)
    {
    	//Send AT asynchronous message
        shell_printf("\r+WIND:keyboard event = %d\n\r", events);
    }
#endif //AT_COMMAND_SHELL
}

/*!*************************************************************************************************
\private
\fn     static void App_UpdateLedState(appDeviceState_t deviceState)
\brief  Called when Application state and LEDs must be updated.

\param  [in]    deviceState    The current device state
***************************************************************************************************/
static void App_UpdateStateLeds
(
    appDeviceState_t deviceState
)
{
    /* If the user presses a button different than the LED off button, reset timestamp */
    if((gpaThrAttr[mThrInstanceId]->devRole == gThrDevRole_Leader_c) &&
       (APP_GetState(mThrInstanceId) != gDeviceState_AppLedOff_c) &&
       (leaderLedTimestamp != 0))
    {
        leaderLedTimestamp = 0;
    }

    APP_SetState(mThrInstanceId, deviceState);

#if VT_KW41Z_MENP
#ifndef RELAY_CLICK_ENABLE
    if(deviceState != gDeviceState_ActiveRouter_c)
    {
      Led_SetState(APP_GetMode(mThrInstanceId), APP_GetState(mThrInstanceId));
    }
#else
    if(deviceState == gDeviceState_NwkConnected_c)
    {
      Led1Off();
    }
#endif /* RELAY_CLICK_ENABLE */
#else
		Led_SetState(APP_GetMode(mThrInstanceId), APP_GetState(mThrInstanceId));
#endif /* VT_KW41Z_MENP */
}

/*!*************************************************************************************************
\private
\fn     static void APP_JoinEventsHandler(thrEvCode_t evCode)
\brief  This function is used to the handle join failed event.

\param  [in]    evCode    Event code
***************************************************************************************************/
static void APP_JoinEventsHandler
(
    thrEvCode_t evCode
)
{
    if(mJoiningIsAppInitiated)
    {
        if(evCode == gThrEv_NwkJoinCnf_Failed_c)
        {
            if(gbRetryInterrupt && !gbCreateNetwork)
            {
                mJoiningIsAppInitiated = TRUE;

                /* Retry to join the network */
                if(THR_NwkJoin(mThrInstanceId, THR_APP_JOIN_DISCOVERY_METHOD) != gThrStatus_Success_c)
                {
                    /* User can treat join failure according to their application */
                }
                return;
            }
            else if(gbCreateNetwork)
            {
#if VT_KW41Z_MENP
                 /* Reset the module */
	         THR_FactoryReset();
#else
                /* Create the network */
                (void)THR_NwkCreate(mThrInstanceId);
#endif /* VT_KW41Z_MENP */
            }
            mJoiningIsAppInitiated = FALSE;
        }
        else if(evCode == gThrEv_NwkJoinCnf_Success_c)
        {
            mJoiningIsAppInitiated = FALSE;
        }
    }
}

/*==================================================================================================
  Coap Demo functions:
==================================================================================================*/
/*!*************************************************************************************************
\private
\fn     static void APP_CoapGenericCallback(coapSessionStatus_t sessionStatus, void *pData,
                                            coapSession_t *pSession, uint32_t dataLen)
\brief  This function is the generic callback function for CoAP message.

\param  [in]    sessionStatus   Status for CoAP session
\param  [in]    pData           Pointer to CoAP message payload
\param  [in]    pSession        Pointer to CoAP session
\param  [in]    dataLen         Length of CoAP payload
***************************************************************************************************/
static void APP_CoapGenericCallback
(
    coapSessionStatus_t sessionStatus,
    void *pData,
    coapSession_t *pSession,
    uint32_t dataLen
)
{
    /* If no ACK was received, try again */
    if(sessionStatus == gCoapFailure_c)
    {
        if(FLib_MemCmp(pSession->pUriPath->pUriPath, (coapUriPath_t *)&gAPP_TEMP_URI_PATH.pUriPath,
                       pSession->pUriPath->length))
        {
            (void)NWKU_SendMsg(APP_ReportTemp, NULL, mpAppThreadMsgQueue);
        }
    }
    /* Process data, if any */
}

/*!*************************************************************************************************
\private
\fn     static void APP_ReportTemp(void *pParam)
\brief  This open a socket and report the temperature to gCoapDestAddress.

\param  [in]    pParam    Not used
***************************************************************************************************/
static void APP_ReportTemp
(
    void *pParam
)
{
    coapSession_t *pSession = NULL;
    /* Get Temperature */
    uint8_t *pTempString = App_GetTempDataString();
    uint32_t ackPloadSize;
    ifHandle_t ifHandle = THR_GetIpIfPtrByInstId(mThrInstanceId);

    if(!IP_IF_IsMyAddr(ifHandle->ifUniqueId, &gCoapDestAddress))
    {
        pSession = COAP_OpenSession(mAppCoapInstId);

        if(NULL != pSession)
        {
            coapMsgTypesAndCodes_t coapMessageType = gCoapMsgTypeNonPost_c;

            pSession->pCallback = NULL;
            FLib_MemCpy(&pSession->remoteAddr, &gCoapDestAddress, sizeof(ipAddr_t));
            ackPloadSize = strlen((char *)pTempString);
            COAP_SetUriPath(pSession, (coapUriPath_t *)&gAPP_TEMP_URI_PATH);

            if(!IP6_IsMulticastAddr(&gCoapDestAddress))
            {
                coapMessageType = gCoapMsgTypeConPost_c;
                pSession->pCallback = APP_CoapGenericCallback;
            }

            COAP_Send(pSession, coapMessageType, pTempString, ackPloadSize);
        }
    }
    /* Print temperature in shell */
    shell_write("\r");
    shell_write((char *)pTempString);
    shell_refresh();
    MEM_BufferFree(pTempString);
}

#if VT_KW41Z_DEMO
/*!*************************************************************************************************
\private
\fn     void APP_Reportkey_event(void *pParam)
\brief  This open a socket and report the temperature to gAppRemoteAddress.

\param  [in]    pParam   pointer to stack event

\return         void
***************************************************************************************************/
static void APP_Reportkey_event(void *pParam)
{

    coapSession_t *pSession = NULL;
    uint8_t *pKeyCount;
    uint32_t ackPloadSize;
    char data_buffer[80];
    ipAddr_t pDestAddr;
    pSession = COAP_OpenSession(mAppCoapInstId);
    
    if (NULL != pSession)
    {
        memset(data_buffer,0,80);
        coapMsgTypesAndCodes_t coapMessageType = gCoapMsgTypeNonPost_c;
        
        pDestAddr.addr8[0]=0xFD;
        pDestAddr.addr8[1]=0x01;
        pDestAddr.addr8[2]=0x00;
        pDestAddr.addr8[3]=0x00;
        pDestAddr.addr8[4]=0x00;
        pDestAddr.addr8[5]=0x00;
        pDestAddr.addr8[6]=0x00;
        pDestAddr.addr8[7]=0x00;
        pDestAddr.addr8[8]=0x00;
        pDestAddr.addr8[9]=0x00;
        pDestAddr.addr8[10]=0x00;
        pDestAddr.addr8[11]=0x00;
        pDestAddr.addr8[12]=0x00;
        pDestAddr.addr8[13]=0x00;
        pDestAddr.addr8[14]=0x00;
        pDestAddr.addr8[15]=0x02;
        
        //pSession->pIfHandle = THR_GetIpIfByInstId(mThrInstanceId);
        pSession->pCallback = NULL;
        FLib_MemCpy(&pSession->remoteAddr, &pDestAddr, sizeof(ipAddr_t));
        /* Get Temperature */
        //key_event_count++;
        sprintf(data_buffer,"{\"eui\":\"%s\",\"key_count\":\"%d\"}",eui_string, key_event_count);
        pKeyCount = (unsigned char *)data_buffer;

        ackPloadSize = strlen((char*)pKeyCount);
        //COAP_AddOptionToList(pSession,COAP_URI_PATH_OPTION, APP_KEY_EVENT_URI_PATH, SizeOfString(APP_KEY_EVENT_URI_PATH));

        /*if(!IP6_IsMulticastAddr(&gCoapDestAddress))
        {
            coapMessageType = gCoapMsgTypeConPost_c;
            pSession->pCallback = APP_CoapGenericCallback;
        }*/

        COAP_Send(pSession, coapMessageType, pKeyCount, ackPloadSize);

    }
}

static void APP_ReportSensors(char* pSensorString)
{
    coapSession_t *pSession = NULL;
    uint32_t ackPloadSize;
    ipAddr_t pDestAddr;
    pSession = COAP_OpenSession(mAppCoapInstId);

    if (NULL != pSession)
    {
        coapMsgTypesAndCodes_t coapMessageType = gCoapMsgTypeNonPost_c;

        pDestAddr.addr8[0]=0xFD;
        pDestAddr.addr8[1]=0x01;
        pDestAddr.addr8[2]=0x00;
        pDestAddr.addr8[3]=0x00;
        pDestAddr.addr8[4]=0x00;
        pDestAddr.addr8[5]=0x00;
        pDestAddr.addr8[6]=0x00;
        pDestAddr.addr8[7]=0x00;
        pDestAddr.addr8[8]=0x00;
        pDestAddr.addr8[9]=0x00;
        pDestAddr.addr8[10]=0x00;
        pDestAddr.addr8[11]=0x00;
        pDestAddr.addr8[12]=0x00;
        pDestAddr.addr8[13]=0x00;
        pDestAddr.addr8[14]=0x00;
        pDestAddr.addr8[15]=0x02;

        //pSession->pIfHandle = THR_GetIpIfByInstId(mThrInstanceId);
        pSession->pCallback = NULL;
        FLib_MemCpy(&pSession->remoteAddr, &pDestAddr, sizeof(ipAddr_t));

        ackPloadSize = strlen(pSensorString);
        //COAP_AddOptionToList(pSession,COAP_URI_PATH_OPTION, APP_KEY_EVENT_URI_PATH, SizeOfString(APP_KEY_EVENT_URI_PATH));

        /*if(!IP6_IsMulticastAddr(&gCoapDestAddress))
        {
            coapMessageType = gCoapMsgTypeConPost_c;
            pSession->pCallback = APP_CoapGenericCallback;
        }*/

        COAP_Send(pSession, coapMessageType, pSensorString, ackPloadSize);
    }
}

#endif /* VT_KW41Z_DEMO */

static void APP_TestCoapMsg(void *pParam)
{

    coapSession_t *pSession = NULL;
    uint32_t ackPloadSize;
    char data_buffer[32];
    ipAddr_t pDestAddr;
    pSession = COAP_OpenSession(mAppCoapInstId);

    if (NULL != pSession)
    {
        memset(data_buffer,0,32);
        coapMsgTypesAndCodes_t coapMessageType = gCoapMsgTypeNonPost_c;

        pDestAddr.addr8[0]=0xFD;
        pDestAddr.addr8[1]=0x01;
        pDestAddr.addr8[2]=0x00;
        pDestAddr.addr8[3]=0x00;
        pDestAddr.addr8[4]=0x00;
        pDestAddr.addr8[5]=0x00;
        pDestAddr.addr8[6]=0x00;
        pDestAddr.addr8[7]=0x00;
        pDestAddr.addr8[8]=0x00;
        pDestAddr.addr8[9]=0x00;
        pDestAddr.addr8[10]=0x00;
        pDestAddr.addr8[11]=0x00;
        pDestAddr.addr8[12]=0x00;
        pDestAddr.addr8[13]=0x00;
        pDestAddr.addr8[14]=0x00;
        pDestAddr.addr8[15]=0x02;

        //pSession->pIfHandle = THR_GetIpIfByInstId(mThrInstanceId);
        pSession->pCallback = NULL;
        FLib_MemCpy(&pSession->remoteAddr, &pDestAddr, sizeof(ipAddr_t));
        sprintf(data_buffer,"ciao pippo");

        ackPloadSize = strlen((char*)data_buffer);
        COAP_AddOptionToList(pSession,COAP_URI_PATH_OPTION, (uint8_t*)"/testcoap", SizeOfString("/testcoap"));

        /*if(!IP6_IsMulticastAddr(&gCoapDestAddress))
        {
            coapMessageType = gCoapMsgTypeConPost_c;
            pSession->pCallback = APP_CoapGenericCallback;
        }*/

        COAP_Send(pSession, coapMessageType, data_buffer, ackPloadSize);

    }
}

#ifndef FRDM_KW41Z
/*!*************************************************************************************************
\private
\fn     static nwkStatus_t APP_SendDataSinkCommand(uint8_t *pCommand, uint8_t dataLen)
\brief  This function is used to send a Data Sink command to APP_DEFAULT_DEST_ADDR.

\param  [in]    pCommand       Pointer to command data
\param  [in]    dataLen        Data length

\return         nwkStatus_t    Status of the command
***************************************************************************************************/
static nwkStatus_t APP_SendDataSinkCommand
(
    uint8_t *pCommand,
    uint8_t dataLen
)
{
    nwkStatus_t status = gNwkStatusFail_c;
    coapSession_t *pSession = COAP_OpenSession(mAppCoapInstId);

    if(pSession)
    {
        ipAddr_t coapDestAddress = APP_DEFAULT_DEST_ADDR;

        pSession->pCallback = NULL;
        FLib_MemCpy(&pSession->remoteAddr, &coapDestAddress, sizeof(ipAddr_t));
        COAP_SetUriPath(pSession, (coapUriPath_t *)&gAPP_SINK_URI_PATH);
        status = COAP_Send(pSession, gCoapMsgTypeNonPost_c, pCommand, dataLen);
    }

    return status;
}

/*!*************************************************************************************************
\private
\fn     static void APP_SendDataSinkCreate(void *pParam)
\brief  This function is used to send a Data Sink Create command to APP_DEFAULT_DEST_ADDR.

\param  [in]    pParam    Not used
***************************************************************************************************/
static void APP_SendDataSinkCreate
(
    void *pParam
)
{
    uint8_t aCommand[] = {"create"};

    /* Send command over the air */
    if(APP_SendDataSinkCommand(aCommand, sizeof(aCommand)) == gNwkStatusSuccess_c)
    {
        /* Local data sink create */
        (void)THR_GetIP6Addr(mThrInstanceId, gMLEIDAddr_c, &gCoapDestAddress, NULL);
    }
}

/*!*************************************************************************************************
\private
\fn     static void APP_SendDataSinkRelease(void *pParam)
\brief  This function is used to send a Data Sink Release command to APP_DEFAULT_DEST_ADDR.

\param  [in]    pParam    Not used
***************************************************************************************************/
static void APP_SendDataSinkRelease
(
    void *pParam
)
{
    uint8_t aCommand[] = {"release"};

    /* Send command over the air */
    if(APP_SendDataSinkCommand(aCommand, sizeof(aCommand)) == gNwkStatusSuccess_c)
    {
        /* Local data sink release */
        APP_LocalDataSinkRelease(pParam);
    }
}
#endif

#if !(VT_KW41Z_MENP)
/*!*************************************************************************************************
\private
\fn     static void APP_SendLedCommand(uint8_t *pCommand, uint8_t dataLen)
\brief  This function is used to send a Led command to gCoapDestAddress.

\param  [in]    pCommand    Pointer to command data
\param  [in]    dataLen     Data length
***************************************************************************************************/
static void APP_SendLedCommand
(
    uint8_t *pCommand,
    uint8_t dataLen
)
{
    ifHandle_t ifHandle = THR_GetIpIfPtrByInstId(mThrInstanceId);

    if(!IP_IF_IsMyAddr(ifHandle->ifUniqueId, &gCoapDestAddress))
    {
        coapSession_t *pSession = COAP_OpenSession(mAppCoapInstId);

        if(pSession)
        {
            coapMsgTypesAndCodes_t coapMessageType = gCoapMsgTypeNonPost_c;

            pSession->pCallback = NULL;
            FLib_MemCpy(&pSession->remoteAddr, &gCoapDestAddress, sizeof(ipAddr_t));
            COAP_SetUriPath(pSession,(coapUriPath_t *)&gAPP_LED_URI_PATH);

            if(!IP6_IsMulticastAddr(&gCoapDestAddress))
            {
                coapMessageType = gCoapMsgTypeConPost_c;
                pSession->pCallback = APP_CoapGenericCallback;
            }
            else
            {
                APP_ProcessLedCmd(pCommand, dataLen);
            }
            COAP_Send(pSession, coapMessageType, pCommand, dataLen);
        }
    }
    else
    {
        APP_ProcessLedCmd(pCommand, dataLen);
    }
}

/*!*************************************************************************************************
\private
\fn     static void APP_SendLedRgbOn(void *pParam)
\brief  This function is used to send a Led RGB On command over the air.

\param  [in]    pParam    Not used
***************************************************************************************************/
static void APP_SendLedRgbOn
(
    void *pParam
)
{
    uint8_t aCommand[] = {"rgb r000 g000 b000"};
    uint8_t redValue, greenValue, blueValue;

    /* Red value on: 0x01 - 0xFF */
    redValue = (uint8_t)NWKU_GetRandomNoFromInterval(0x01, THR_ALL_FFs8);

    /* Green value on: 0x01 - 0xFF */
    greenValue = (uint8_t)NWKU_GetRandomNoFromInterval(0x01, THR_ALL_FFs8);

    /* Blue value on: 0x01 - 0xFF */
    blueValue = (uint8_t)NWKU_GetRandomNoFromInterval(0x01, THR_ALL_FFs8);

    NWKU_PrintDec(redValue, aCommand + 5, 3, TRUE);     //aCommand + strlen("rgb r")
    NWKU_PrintDec(greenValue, aCommand + 10, 3, TRUE);  //aCommand + strlen("rgb r000 g")
    NWKU_PrintDec(blueValue, aCommand + 15, 3, TRUE);   //aCommand + strlen("rgb r000 g000 b")

    APP_SendLedCommand(aCommand, sizeof(aCommand));
}

/*!*************************************************************************************************
\private
\fn     static void APP_SendLedRgbOff(void *pParam)
\brief  This function is used to send a Led RGB Off command over the air.

\param  [in]    pParam    Not used
***************************************************************************************************/
static void APP_SendLedRgbOff
(
    void *pParam
)
{
    uint8_t aCommand[] = {"rgb r000 g000 b000"};

    APP_SendLedCommand(aCommand, sizeof(aCommand));
}

/*!*************************************************************************************************
\private
\fn     static void APP_SendLedFlash(void *pParam)
\brief  This function is used to send a Led flash command over the air.

\param  [in]    pParam    Not used
***************************************************************************************************/
static void APP_SendLedFlash
(
    void *pParam
)
{
    uint8_t aCommand[] = {"flash"};

    APP_SendLedCommand(aCommand, sizeof(aCommand));
}

/*!*************************************************************************************************
\private
\fn     static void APP_SendLedColorWheel(void *pParam)
\brief  This function is used to send a Led color wheel command over the air.

\param  [in]    pParam    Not used
***************************************************************************************************/
static void APP_SendLedColorWheel
(
    void *pParam
)
{
    uint8_t aCommand[] = {"color wheel"};

    APP_SendLedCommand(aCommand, sizeof(aCommand));
}
#endif /* VT_KW41Z_MENP */

/*!*************************************************************************************************
\private
\fn     static void APP_LocalDataSinkRelease(void *pParam)
\brief  This function is used to restore the default destination address for CoAP messages.

\param  [in]    pParam    Not used
***************************************************************************************************/
static void APP_LocalDataSinkRelease
(
    void *pParam
)
{
    ipAddr_t defaultDestAddress = APP_DEFAULT_DEST_ADDR;

    FLib_MemCpy(&gCoapDestAddress, &defaultDestAddress, sizeof(ipAddr_t));
}

/*!*************************************************************************************************
\private
\fn     static void APP_CoapLedCb(coapSessionStatus_t sessionStatus, void *pData,
                                  coapSession_t *pSession, uint32_t dataLen)
\brief  This function is the callback function for CoAP LED message.
\brief  It performs the required operations and sends back a CoAP ACK message.

\param  [in]    sessionStatus   Status for CoAP session
\param  [in]    pData           Pointer to CoAP message payload
\param  [in]    pSession        Pointer to CoAP session
\param  [in]    dataLen         Length of CoAP payload
***************************************************************************************************/
static void APP_CoapLedCb
(
    coapSessionStatus_t sessionStatus,
    void *pData,
    coapSession_t *pSession,
    uint32_t dataLen
)
{
    /* Process the command only if it is a POST method */
    if((pData) && (sessionStatus == gCoapSuccess_c) && (pSession->code == gCoapPOST_c))
    {
        APP_ProcessLedCmd(pData, dataLen);
    }

    /* Send the reply if the status is Success or Duplicate */
    if((gCoapFailure_c != sessionStatus) && (gCoapConfirmable_c == pSession->msgType))
    {
        /* Send CoAP ACK */
        COAP_Send(pSession, gCoapMsgTypeAckSuccessChanged_c, NULL, 0);
    }
}

/*!*************************************************************************************************
\private
\fn     static void APP_ProcessLedCmd(uint8_t *pCommand, uint8_t dataLen)
\brief  This function is used to process a LED command (on, off, flash, toggle, rgb, color wheel).

\param  [in]    pCommand    Pointer to command data
\param  [in]    dataLen     Data length
***************************************************************************************************/
static void APP_ProcessLedCmd
(
    uint8_t *pCommand,
    uint8_t dataLen
)
{
#if 0
    mFirstPushButtonPressed  = FALSE;

    /* Process command */
    if(FLib_MemCmp(pCommand, "on", 2))
    {
#if VT_KW41Z_MENP
        Led1On();
#else
        App_UpdateStateLeds(gDeviceState_AppLedOn_c);
#endif /* VT_KW41Z_MENP */
    }
    else if(FLib_MemCmp(pCommand, "off", 3))
    {
#if VT_KW41Z_MENP
        Led1Off();
#else
        App_UpdateStateLeds(gDeviceState_AppLedOff_c);
#endif /* VT_KW41Z_MENP */
    }
    else if(FLib_MemCmp(pCommand, "toggle", 6))
    {
        App_UpdateStateLeds(gDeviceState_AppLedToggle_c);
    }
    else if(FLib_MemCmp(pCommand, "flash", 5))
    {
        App_UpdateStateLeds(gDeviceState_AppLedFlash_c);
    }
    else if(FLib_MemCmp(pCommand, "rgb", 3))
    {
        char* p = (char *)pCommand + strlen("rgb");
        uint8_t redValue = 0, greenValue = 0, blueValue = 0;
        appDeviceState_t appState = gDeviceState_AppLedRgb_c;

        dataLen -= strlen("rgb");

        while(dataLen != 0)
        {
            if(*p == 'r')
            {
                p++;
                dataLen--;
                redValue = NWKU_atoi(p);
            }

            if(*p == 'g')
            {
                p++;
                dataLen--;
                greenValue = NWKU_atoi(p);
            }

            if(*p == 'b')
            {
                p++;
                dataLen--;
                blueValue = NWKU_atoi(p);
            }

            dataLen--;
            p++;
        }

        /* Update RGB values */
#if gLedRgbEnabled_d
        Led_UpdateRgbState(redValue, greenValue, blueValue);
#else
        appState = gDeviceState_AppLedOff_c;

        if(redValue || greenValue || blueValue)
        {
            appState = gDeviceState_AppLedOn_c;
        }
#endif
        App_UpdateStateLeds(appState);

        /* If device is leader and has received a RGB LED off command and there were no previous button presses */
        if((gpaThrAttr[mThrInstanceId]->devRole == gThrDevRole_Leader_c) &&
           (!redValue && !greenValue && !blueValue) && (leaderLedTimestamp == 0))
        {
            leaderLedTimestamp = (TMR_GetTimestamp()/1000000) + gAppRestoreLeaderLedTimeout_c;
        }
    }
    else if(FLib_MemCmp(pCommand, "color wheel", 11))
    {
#if gLedRgbEnabled_d
        App_UpdateStateLeds(gDeviceState_AppLedColorWheel_c);
#else
        App_UpdateStateLeds(gDeviceState_AppLedFlash_c);
#endif
    }
#else //0
	char *p;
	uint8_t phase;	//0: '{'; 1: field; 2: ':'; 3: value
    uint8_t redValue = 0, greenValue = 0, blueValue = 0;
    uint8_t* pField;

    /* Set mode state */
    APP_SetMode(mThrInstanceId, gDeviceMode_Application_c);

    p = (char*)pCommand;
    phase = 0;

	m_bUserLedUpdate = false;

    while(dataLen > 0 && *p != '}')
    {
    	if (*p == ' ' || *p=='"')
    	{
    		p++;
    		dataLen--;
    		continue;
    	}

    	switch(phase)
    	{
    		case 0:
    			if (*p == '{')
    			{
    				phase = 1;
    			}
    			else
    			{
    				return;
    			}
    			break;

    		case 1:
    			if (*p == 'r')
    			{
    				pField = &redValue;
    				phase = 2;
    			}
    			else if (*p == 'g')
    			{
    				pField = &greenValue;
    				phase = 2;
    			}
    			else if (*p == 'b')
    			{
    				pField = &blueValue;
    				phase = 2;
    			}
    			else
    			{
    				return;
    			}
    			break;

    		case 2:
    			if (*p == ':')
    			{
    				phase = 3;
    			}
    			else
    			{
    				return;
    			}
    			break;

    		case 3:
    			if (*p >= '0' && *p <= '9')
    			{
    				*pField = (*pField)*10 + (*p - '0');
    			}
    			else if (*p == ',')
    			{
    				phase = 1;
    			}
    			else
    			{
    				return;
    			}
    			break;
    	}

		p++;
		dataLen--;
    }

	m_bUserLedUpdate = true;
	m_redValue = redValue;
	m_greenValue = greenValue;
	m_blueValue = blueValue;
#endif //0
}

#if VT_KW41Z_DEMO
/*!*************************************************************************************************
\private
\fn     void APP_CoapkeyeventCb(bool_t sessionStatus, void *pData, coapSession_t *pSession, uint32_t dataLen)
\brief  This function is the callback function for CoAP message. It sends the KeyCount value in
a CoAP ACK message.

\param  [in]    sessionStatus   status for CoAP session
\param  [in]    pData           pointer to CoAP message payload
\param  [in]    pSession        pointer to CoAP session
\param  [in]    dataLen         length of CoAP payload

\return         void
***************************************************************************************************/
static void APP_CoapkeyeventCb
(
    coapSessionStatus_t sessionStatus,
    void *pData,
    coapSession_t *pSession,
    uint32_t dataLen
)
{
    uint8_t *pKeyEvent = NULL;
    uint32_t ackPloadSize = 0;
    char data_buffer[32];
    
    /* Send CoAP ACK */
    if (gCoapGET_c == pSession->code)
    {
        memset(data_buffer,0,32);
        sprintf(data_buffer,"key count : %d",key_event_count);
        pKeyEvent = (uint8_t *)&data_buffer[0];
        ackPloadSize = strlen((char*)pKeyEvent);
    }
    if (gCoapConfirmable_c == pSession->msgType)
    {
        if (gCoapGET_c == pSession->code)
        {
            COAP_Send(pSession, gCoapMsgTypeAckSuccessChanged_c, pKeyEvent, ackPloadSize);
        }
        else
        {
            COAP_Send(pSession, gCoapMsgTypeAckSuccessChanged_c, NULL, 0);
        }
    }

}

/*!*************************************************************************************************
\private
\fn     void APP_CoapresetCb(bool_t sessionStatus, void *pData, coapSession_t *pSession, uint32_t dataLen)
\brief  This function is the callback function for CoAP message. It performs the Factory reset
operation and sends back a CoAP ACK message.

\param  [in]    sessionStatus   status for CoAP session
\param  [in]    pData           pointer to CoAP message payload
\param  [in]    pSession        pointer to CoAP session
\param  [in]    dataLen         length of CoAP payload

\return         void
***************************************************************************************************/
static void APP_CoapresetCb
(
    coapSessionStatus_t sessionStatus,
    void *pData,
    coapSession_t *pSession,
    uint32_t dataLen
)
{
    /* Process the command only if it is a POST method */
    if ((sessionStatus == gCoapSuccess_c) && (pSession->code == gCoapPOST_c))
    {
        shell_write("Factory reset\r\n");
        THR_FactoryReset();
    }

    /* Send the reply if the status is Success or Duplicate */
    if ((gCoapFailure_c != sessionStatus) && (gCoapConfirmable_c == pSession->msgType))
    {
        /* Send CoAP ACK */
        COAP_Send(pSession, gCoapMsgTypeAckSuccessChanged_c, NULL, 0);
    }
}
#endif /* VT_KW41Z_DEMO */

/*!*************************************************************************************************
\private
\fn     static void APP_CoapTempCb(coapSessionStatus_t sessionStatus, void *pData,
                                   coapSession_t *pSession, uint32_t dataLen)
\brief  This function is the callback function for CoAP temperature message.
\brief  It sends the temperature value in a CoAP ACK message.

\param  [in]    sessionStatus   Status for CoAP session
\param  [in]    pData           Pointer to CoAP message payload
\param  [in]    pSession        Pointer to CoAP session
\param  [in]    dataLen         Length of CoAP payload
***************************************************************************************************/
static void APP_CoapTempCb
(
    coapSessionStatus_t sessionStatus,
    void *pData,
    coapSession_t *pSession,
    uint32_t dataLen
)
{
    uint8_t *pTempString = NULL;
    uint32_t ackPloadSize = 0, maxDisplayedString = 10;

    /* Send CoAP ACK */
    if(gCoapGET_c == pSession->code)
    {
        /* Get Temperature */
        pTempString = App_GetTempDataString();
        ackPloadSize = strlen((char*)pTempString);
    }
    /* Do not parse the message if it is duplicated */
    else if((gCoapPOST_c == pSession->code) && (sessionStatus == gCoapSuccess_c))
    {
        if(NULL != pData)
        {
            char addrStr[INET6_ADDRSTRLEN];
            uint8_t temp[10];

            ntop(AF_INET6, &pSession->remoteAddr, addrStr, INET6_ADDRSTRLEN);
            shell_write("\r");

            if(0 != dataLen)
            {
                /* Prevent from buffer overload */
                (dataLen >= maxDisplayedString) ? (dataLen = (maxDisplayedString - 1)) : (dataLen);
                temp[dataLen]='\0';
                FLib_MemCpy(temp,pData,dataLen);
                shell_printf((char*)temp);
            }
            shell_printf("\tFrom IPv6 Address: %s\n\r", addrStr);
            shell_refresh();
#ifdef VT_KW41Z_DEMO
            FLib_MemSet(addrStr, 0,INET6_ADDRSTRLEN );

            ntop(AF_INET6, &in6addr_realmlocal_allthreadnodes, addrStr, INET6_ADDRSTRLEN);
            shell_write("\r");
            shell_printf("\t IPv6 readlm: %s\n\r", addrStr);
            shell_refresh();
            shell_printf("\t IPv6 readlm0: %2X,%2X,%2X,%2X\n\r", in6addr_realmlocal_allthreadnodes.addr8[0],in6addr_realmlocal_allthreadnodes.addr8[1], in6addr_realmlocal_allthreadnodes.addr8[2], in6addr_realmlocal_allthreadnodes.addr8[4]);
            shell_printf("\t IPv6 readlm1: %2X,%2X,%2X,%2X\n\r", in6addr_realmlocal_allthreadnodes.addr8[4],in6addr_realmlocal_allthreadnodes.addr8[5], in6addr_realmlocal_allthreadnodes.addr8[6], in6addr_realmlocal_allthreadnodes.addr8[7]);
            shell_printf("\t IPv6 readlm2: %2X,%2X,%2X,%2X\n\r", in6addr_realmlocal_allthreadnodes.addr8[8],in6addr_realmlocal_allthreadnodes.addr8[9], in6addr_realmlocal_allthreadnodes.addr8[10], in6addr_realmlocal_allthreadnodes.addr8[11]);
            shell_printf("\t IPv6 readlm3: %2X,%2X,%2X,%2X\n\r", in6addr_realmlocal_allthreadnodes.addr8[12],in6addr_realmlocal_allthreadnodes.addr8[13], in6addr_realmlocal_allthreadnodes.addr8[14], in6addr_realmlocal_allthreadnodes.addr8[15]);
            shell_refresh();
#endif            
        }
    }

    if(gCoapConfirmable_c == pSession->msgType)
    {
        if(gCoapGET_c == pSession->code)
        {
            COAP_Send(pSession, gCoapMsgTypeAckSuccessChanged_c, pTempString, ackPloadSize);
        }
        else
        {
            COAP_Send(pSession, gCoapMsgTypeAckSuccessChanged_c, NULL, 0);
        }
    }

    if(pTempString)
    {
        MEM_BufferFree(pTempString);
    }
}

/*!*************************************************************************************************
\private
\fn     static void APP_CoapSinkCb(coapSessionStatus_t sessionStatus, void *pData,
                                   coapSession_t *pSession, uint32_t dataLen)
\brief  This function is the callback function for CoAP sink message.

\param  [in]    sessionStatus   Status for CoAP session
\param  [in]    pData           Pointer to CoAP message payload
\param  [in]    pSession        Pointer to CoAP session
\param  [in]    dataLen         Length of CoAP payload
***************************************************************************************************/
static void APP_CoapSinkCb
(
    coapSessionStatus_t sessionStatus,
    void *pData,
    coapSession_t *pSession,
    uint32_t dataLen
)
{
    /* Do not execute the command multiple times, if the received message is duplicated */
    if((pData) && (sessionStatus == gCoapSuccess_c))
    {
        /* Process command */
        if(FLib_MemCmp(pData, "create", 6))
        {
            /* Data sink create */
            FLib_MemCpy(&gCoapDestAddress, &pSession->remoteAddr, sizeof(ipAddr_t));
        }

        if(FLib_MemCmp(pData, "release", 7))
        {
            /* Data sink release */
            APP_LocalDataSinkRelease(NULL);
        }
    }

    if(gCoapConfirmable_c == pSession->msgType)
    {
        /* Send CoAP ACK */
        COAP_Send(pSession, gCoapMsgTypeAckSuccessChanged_c, NULL, 0);
    }
}

/*!*************************************************************************************************
\private
\fn     static void App_RestoreLeaderLed(void *param)
\brief  Called in Application state to restore leader LED.

\param  [in]    param    Not used
***************************************************************************************************/
static void App_RestoreLeaderLed
(
    void *param
)
{
    App_UpdateStateLeds(gDeviceState_Leader_c);
}

#if LARGE_NETWORK
/*!*************************************************************************************************
\private
\fn     static void APP_SendResetToFactoryCommand(void *pParam)
\brief  This function is used to send a Factory Reset command to APP_DEFAULT_DEST_ADDR.

\param  [in]    pParam    Pointer to stack event
***************************************************************************************************/
static void APP_SendResetToFactoryCommand
(
    void *param
)
{
    coapSession_t *pSession = COAP_OpenSession(mAppCoapInstId);

    if(pSession)
    {
        ipAddr_t coapDestAddress = APP_DEFAULT_DEST_ADDR;

        pSession->pCallback = NULL;
        FLib_MemCpy(&pSession->remoteAddr, &coapDestAddress, sizeof(ipAddr_t));
        COAP_SetUriPath(pSession, (coapUriPath_t *)&gAPP_RESET_URI_PATH);
        COAP_Send(pSession, gCoapMsgTypeNonPost_c, NULL, 0);
    }
}

/*!*************************************************************************************************
\private
\fn     static void APP_CoapResetToFactoryDefaultsCb(coapSessionStatus_t sessionStatus, void *pData,
                                                     coapSession_t *pSession, uint32_t dataLen)
\brief  This function is the callback function for CoAP factory reset message.

\param  [in]    sessionStatus   Status for CoAP session
\param  [in]    pData           Pointer to CoAP message payload
\param  [in]    pSession        Pointer to CoAP session
\param  [in]    dataLen         Length of CoAP payload
***************************************************************************************************/
static void APP_CoapResetToFactoryDefaultsCb
(
    coapSessionStatus_t sessionStatus,
    void *pData,
    coapSession_t *pSession,
    uint32_t dataLen
)
{
    THR_FactoryReset();
}
#endif

#if APP_AUTOSTART
/*!*************************************************************************************************
\private
\fn     static void APP_AutoStart(void)
\brief  This is the autostart function, used to start the network joining.

\param  [in]    param    Not used
***************************************************************************************************/
static void APP_AutoStart
(
    void *param
)
{
    if(!THR_GetAttr_IsDevConnected(mThrInstanceId))
    {
        mJoiningIsAppInitiated = TRUE;

        if(THR_NwkJoin(mThrInstanceId, THR_APP_JOIN_DISCOVERY_METHOD) != gThrStatus_Success_c)
        {
            /* User can treat join failure according to their application */
        }
    }
}

/*!*************************************************************************************************
\private
\fn     static void APP_AutoStartCb(void)
\brief  This is the autostart callback function.

\param  [in]    param    Not used
***************************************************************************************************/
static void APP_AutoStartCb
(
    void *param
)
{
    NWKU_SendMsg(APP_AutoStart, NULL, mpAppThreadMsgQueue);
}
#endif

/*==================================================================================================
Private debug functions
==================================================================================================*/

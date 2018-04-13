/*==================================================================================================
Include Files
==================================================================================================*/
/*  SDK Included Files */
#include "app_FXLS8972CF_accelerometer.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_i2c.h"

#include "pin_mux.h"
#include "fsl_gpio.h"
#include "fsl_port.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define FXLS8972CF_ADDR		0x18

#define OUT_X_LSB_REG		0x04
#define OUT_X_MSB_REG		0x05
#define OUT_Y_LSB_REG		0x06
#define OUT_Y_MSB_REG		0x07
#define OUT_Z_LSB_REG		0x08
#define OUT_Z_MSB_REG		0x09
#define WHO_AM_I_REG		0x13
#define SYS_MODE_REG		0x14
enum
{
	accSTDBY 	= 0x00,
	accWAKE		= 0x01,
	accSLEEP	= 0x02,
	accEXT_TRG	= 0x03
};
#define SENS_CONFIG1_REG	0x15

static float fxls_bit_res;

static bool WriteAccelReg(uint8_t reg_addr, uint8_t value)
{
    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress = FXLS8972CF_ADDR;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = reg_addr;
    masterXfer.subaddressSize = 1;
    masterXfer.data = &value;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    if(I2C_MasterTransferBlocking(BOARD_ACCEL_I2C_BASEADDR, &masterXfer) == kStatus_Success)
    {
      return true;
    }

	return false;
}

static bool ReadAccelRegs(uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize)
{
    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress = FXLS8972CF_ADDR;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = reg_addr;
    masterXfer.subaddressSize = 1;
    masterXfer.data = rxBuff;
    masterXfer.dataSize = rxSize;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    if(I2C_MasterTransferBlocking(BOARD_ACCEL_I2C_BASEADDR, &masterXfer) == kStatus_Success)
    {
      return true;
    }

	return false;
}

void accelerometer_FXLS8972_Init()
{
	uint8_t who_am_i;

	ReadAccelRegs(WHO_AM_I_REG, &who_am_i, 1);

	//Set ACTIVE - FS=2g
	WriteAccelReg(SENS_CONFIG1_REG, 1);
	fxls_bit_res = 0.00098;
}

void accelerometer_FXLS8972_get_data_string(char *data)
{
	uint8_t sys_mode;
	uint8_t retry = 0;
	int16_t xAccelData;
	int16_t yAccelData;
	int16_t zAccelData;
	uint8_t rawData[6];

	ReadAccelRegs(SYS_MODE_REG, &sys_mode, 1);

	while((sys_mode&0x03) != accWAKE && retry < 10)
	{
		//Set ACTIVE
		WriteAccelReg(SENS_CONFIG1_REG, 1);
		retry++;
		OSA_TimeDelay(100);
	}

	xAccelData = 0;
	yAccelData = 0;
	zAccelData = 0;

	if (retry < 10)
	{
		ReadAccelRegs(OUT_X_LSB_REG, rawData, 6);
		xAccelData = ((int16_t)(((rawData[1] * 256U) | rawData[0])));
		yAccelData = ((int16_t)(((rawData[3] * 256U) | rawData[2])));
		zAccelData = ((int16_t)(((rawData[5] * 256U) | rawData[4])));
	}

    sprintf((char*)data, "\"Ax\":\"%.3f\",\"Ay\":\"%.3f\",\"Az\":\"%.3f\"", xAccelData*fxls_bit_res, yAccelData*fxls_bit_res, zAccelData*fxls_bit_res);

}

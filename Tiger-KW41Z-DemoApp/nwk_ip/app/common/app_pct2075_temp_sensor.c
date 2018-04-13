#include "app_pct2075_temp_sensor.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_i2c.h"

#define PCT2075_I2C_ADDRESS		0x48
#define TEMPERATURE_POINTER		0

int16_t pct2075_read_temperature(void)
{
    i2c_master_transfer_t masterXfer;
    uint8_t cData[2];
    uint16_t wRet = 0;

    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress = PCT2075_I2C_ADDRESS;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = TEMPERATURE_POINTER;
    masterXfer.subaddressSize = 1;
    masterXfer.data = cData;
    masterXfer.dataSize = 2;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    if(I2C_MasterTransferBlocking(I2C0, &masterXfer) == kStatus_Success)
    {
    	wRet = (cData[1]>>5) | (((uint16_t)cData[0])<<3);
    	if (cData[0] & 0x80)
    	{
    		//sign extension
    		wRet |= 0xF800;
    	}
    }

    return (int16_t)wRet;
}

void pct2075_get_temperature_string(char *datastr)
{
	int16_t temperature;

	temperature = pct2075_read_temperature();

    sprintf((char*)datastr, "\"temperature\":\"%.3f\"", temperature*(float)0.125);
}

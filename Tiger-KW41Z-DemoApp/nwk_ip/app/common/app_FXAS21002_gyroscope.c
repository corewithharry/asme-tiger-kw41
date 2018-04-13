#include "app_FXAS21002_gyroscope.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_i2c.h"

#define FXAS21002C_ADDRESS 0x21

#define STATUS           	0x00
#define DR_STATUS        	0x07
#define F_STATUS         	0x08
#define OUT_X_MSB        	0x01
#define OUT_X_LSB        	0x02
#define OUT_Y_MSB        	0x03
#define OUT_Y_LSB        	0x04
#define OUT_Z_MSB        	0x05
#define OUT_Z_LSB        	0x06
#define F_SETUP          	0x09
#define F_EVENT          	0x0A
#define INT_SRC_FLAG     	0x0B
	#define BOOTEND_B		0x08
#define WHO_AM_I         	0x0C
#define CTRL_REG0        	0x0D
#define RT_CFG           	0x0E
#define RT_SRC           	0x0F
#define RT_THS           	0x10
#define RT_COUNT         	0x11
#define TEMP             	0x12
#define CTRL_REG1        	0x13
	#define RST_B			0x40
	#define ACTIVE_B	 	0x02
	#define READY_B		 	0x01
#define CTRL_REG2        	0x14
#define CTRL_REG3        	0x15

enum gyroFSR {
  GFS_2000DPS = 0,
  GFS_1000DPS,
  GFS_500DPS,
  GFS_250DPS
};

enum gyroODR {
  ODR_800HZ = 0, // 200 Hz
  ODR_400HZ,
  ODR_200HZ,
  ODR_100HZ,
  ODR_50HZ,
  ODR_25HZ,
  ODR_12_5HZ,
  ODR_12_5HZ_2,
};

static float gyro_bit_res;

static uint8_t write_reg(uint8_t subAddress, uint8_t data)
{
    i2c_master_transfer_t masterXfer;

    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress = FXAS21002C_ADDRESS;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = subAddress;
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

static uint8_t read_regs(uint8_t subAddress, uint8_t* pcData, uint8_t datalen)
{
    i2c_master_transfer_t masterXfer;

    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress = FXAS21002C_ADDRESS;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = subAddress;
    masterXfer.subaddressSize = 1;
    masterXfer.data = pcData;
    masterXfer.dataSize = datalen;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    if(I2C_MasterTransferBlocking(I2C0, &masterXfer) == kStatus_Success)
    {
    	return 1;
    }

    return 0;
}

// Sets the FXAS21000 to standby mode.
// It must be in standby to change most register settings
static uint8_t FXAS21002CStandby(void)
{
	uint8_t ctrl_reg;

	if (!read_regs(CTRL_REG1, &ctrl_reg, 1))
	{
		return 0;
	}

	ctrl_reg &= ~(ACTIVE_B | READY_B);

	if (!write_reg(CTRL_REG1, ctrl_reg))
	{
		return 0;
	}

	return 1;
}

// Sets the FXAS21000 to active mode.
// Needs to be in this mode to output data
static uint8_t FXAS21002CReady(void)
{
	uint8_t ctrl_reg;

	if (!read_regs(CTRL_REG1, &ctrl_reg, 1))
	{
		return 0;
	}

	ctrl_reg &= ~ACTIVE_B;
	ctrl_reg |= READY_B;

	if (!write_reg(CTRL_REG1, ctrl_reg))
	{
		return 0;
	}

	return 1;
}

// Sets the FXAS21000 to active mode.
// Needs to be in this mode to output data
static uint8_t FXAS21002CActive()
{
	uint8_t ctrl_reg;

	if (!read_regs(CTRL_REG1, &ctrl_reg, 1))
	{
		return 0;
	}

	ctrl_reg |= ACTIVE_B;

	if (!write_reg(CTRL_REG1, ctrl_reg))
	{
		return 0;
	}

	return 1;
}

static uint8_t FXAS21002CReset(void)
{
	uint8_t int_src_reg;

	if (!write_reg(CTRL_REG1, RST_B))
	{
		return 0;
	}

	do
	{
		for(int i=0; i<10000; i++)
		{
			__asm("NOP");
		}

		if (!read_regs(INT_SRC_FLAG, &int_src_reg, 1))
		{
			return 0;
		}
	}while((int_src_reg & BOOTEND_B) == 0);

	return 1;
}

uint8_t gyroscope_init(void)
{
	uint8_t who_am_i;

	if (!read_regs(WHO_AM_I, &who_am_i, 1))
	{
		return 0;
	}

	if (who_am_i != 0xD7)
	{
		return 0;
	}

	if (!FXAS21002CReset())
	{
		return 0;
	}

	if (!FXAS21002CStandby())	// Must be in standby to change registers
	{
		return 0;
	}

	// Set up the full scale range
	if (!write_reg(CTRL_REG0, GFS_250DPS))
	{
		return 0;
	}
	gyro_bit_res = 0.0078125;

	// Setup ODR
	if (!write_reg(CTRL_REG1, ODR_50HZ<<2))
	{
		return 0;
	}

  // Disable FIFO, enable data ready interrupt, route to INT1
  // Active LOW, open drain output on interrupts
	if (!write_reg(CTRL_REG2, 0x0D))
	{
		return 0;
	}

  // Set up rate threshold detection; at max rate threshold = FSR; rate threshold = THS*FSR/128
  //writeByte(FXAS21002C_ADDRESS, RT_CFG, 0x07);         // enable rate threshold detection on all axes
  //writeByte(FXAS21002C_ADDRESS, RT_THS, 0x00 | 0x0D);  // unsigned 7-bit THS, set to one-tenth FSR; set clearing debounce counter
  //writeByte(FXAS21002C_ADDRESS, RT_COUNT, 0x04);       // set to 4 (can set up to 255)

	if (!FXAS21002CActive())  // Set to active to start reading
	{
		return 0;
	}

	return 1;
}

uint8_t gyroscope_read_data(int16_t* panData)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here

  panData[0] = 0;
  panData[1] = 0;
  panData[2] = 0;

  if (read_regs(OUT_X_MSB, rawData, 6))
  {
	  panData[0] = ((int16_t) rawData[0] << 8 | rawData[1]);
	  panData[1] = ((int16_t) rawData[2] << 8 | rawData[3]);
	  panData[2] = ((int16_t) rawData[4] << 8 | rawData[5]);

	  return 1;
  }

  return 0;
}

void gyroscope_get_data_string(char *datastr)
{
	int16_t data[3];

	gyroscope_read_data(data);

    sprintf((char*)datastr, "\"dpsx\":\"%.3f\",\"dpsy\":\"%.3f\",\"dpsz\":\"%.3f\"", data[0]*gyro_bit_res, data[1]*gyro_bit_res, data[2]*gyro_bit_res);
}

uint8_t gyroscope_read_temperature(int8_t* pnTemp)
{
	uint8_t temp_reg = 0;

	if (read_regs(TEMP, (uint8_t*)pnTemp, 1))
	{
	  return 1;
	}

	return 0;
}


/*  SDK Included Files */
#include "app_gpio_expander.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_i2c.h"

#include "pin_mux.h"
#include "fsl_gpio.h"
#include "fsl_port.h"

#define GPIO_EXP_ADDRESS		0x20

#define INPUT_PORT0_REG			0x00
#define INPUT_PORT1_REG			0x01
#define OUTPUT_PORT0_REG		0x02
#define OUTPUT_PORT1_REG		0x03
#define POLINV_PORT0_REG		0x04
#define POLINV_PORT1_REG		0x05
#define CONFIG_PORT0_REG		0x06
#define CONFIG_PORT1_REG		0x07
//TODO: Add Agile IO registers

#define KEY_PB2		0x80
#define KEY_PB1		0x40

#define LED_BLUE	0x7FFF
#define LED_GREEN	0xBFFF
#define LED_RED		0xDFFF

static union
{
	uint8_t shadow_reg8[2];
	uint16_t shadow_reg16;
}m_uOutputShadowReg;

static uint8_t write_reg16(uint8_t subAddress, uint16_t data_reg)
{
    i2c_master_transfer_t masterXfer;
    uint8_t wr_data[2];

    wr_data[0] = (uint8_t)(data_reg & 0xFF);
    wr_data[1] = (uint8_t)(data_reg >> 8);

    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress = GPIO_EXP_ADDRESS;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = subAddress;
    masterXfer.subaddressSize = 1;
    masterXfer.data = wr_data;
    masterXfer.dataSize = 2;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    if(I2C_MasterTransferBlocking(I2C0, &masterXfer) == kStatus_Success)
    {
    	return 1;
    }

    return 0;
}

static uint8_t write_reg8(uint8_t subAddress, uint8_t data_reg)
{
    i2c_master_transfer_t masterXfer;

    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress = GPIO_EXP_ADDRESS;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = subAddress;
    masterXfer.subaddressSize = 1;
    masterXfer.data = &data_reg;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    if(I2C_MasterTransferBlocking(I2C0, &masterXfer) == kStatus_Success)
    {
    	return 1;
    }

    return 0;
}

static uint8_t read_reg8(uint8_t subAddress, uint8_t* p_data_reg)
{
    i2c_master_transfer_t masterXfer;

    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress = GPIO_EXP_ADDRESS;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = subAddress;
    masterXfer.subaddressSize = 1;
    masterXfer.data = p_data_reg;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    if(I2C_MasterTransferBlocking(I2C0, &masterXfer) == kStatus_Success)
    {
    	return 1;
    }

    return 0;
}

uint8_t gpio_expander_get_keys(void)
{
	uint8_t input0;
	uint8_t cRet = 0;

	read_reg8(INPUT_PORT0_REG, &input0);

	if ((input0 & KEY_PB2) == 0)
	{
		cRet |= USR_PB2_FLAG;
	}

	if ((input0 & KEY_PB1) == 0)
	{
		cRet |= USR_PB1_FLAG;
	}

	return cRet;
}

uint8_t gpio_expander_set_led_color(eLEDColor color)
{
	switch(color)
	{
		case ledOff:
			m_uOutputShadowReg.shadow_reg16 |= ~(LED_BLUE & LED_GREEN & LED_RED);
			break;

		case ledRed:
			m_uOutputShadowReg.shadow_reg16 |= ~(LED_BLUE & LED_GREEN & LED_RED);
			m_uOutputShadowReg.shadow_reg16 &= LED_RED;
			break;

		case ledGreen:
			m_uOutputShadowReg.shadow_reg16 |= ~(LED_BLUE & LED_GREEN & LED_RED);
			m_uOutputShadowReg.shadow_reg16 &= LED_GREEN;
			break;

		case ledBlue:
			m_uOutputShadowReg.shadow_reg16 |= ~(LED_BLUE & LED_GREEN & LED_RED);
			m_uOutputShadowReg.shadow_reg16 &= LED_BLUE;
			break;

		case ledCyan:
			m_uOutputShadowReg.shadow_reg16 |= ~(LED_BLUE & LED_GREEN & LED_RED);
			m_uOutputShadowReg.shadow_reg16 &= (LED_BLUE & LED_GREEN);
			break;

		case ledMagenta:
			m_uOutputShadowReg.shadow_reg16 |= ~(LED_BLUE & LED_GREEN & LED_RED);
			m_uOutputShadowReg.shadow_reg16 &= (LED_BLUE & LED_RED);
			break;

		case ledYellow:
			m_uOutputShadowReg.shadow_reg16 |= ~(LED_BLUE & LED_GREEN & LED_RED);
			m_uOutputShadowReg.shadow_reg16 &= (LED_RED & LED_GREEN);
			break;

		case ledWhite:
			m_uOutputShadowReg.shadow_reg16 &= (LED_BLUE & LED_GREEN & LED_RED);
			break;

		default:
			return 0;
	}

	write_reg16(OUTPUT_PORT0_REG, m_uOutputShadowReg.shadow_reg16);
	return 1;
}

uint8_t gpio_expander_add_led_color(eLEDColor color)
{
	switch(color)
	{
		case ledRed:
			m_uOutputShadowReg.shadow_reg16 &= LED_RED;
			break;

		case ledGreen:
			m_uOutputShadowReg.shadow_reg16 &= LED_GREEN;
			break;

		case ledBlue:
			m_uOutputShadowReg.shadow_reg16 &= LED_BLUE;
			break;

		case ledCyan:
			m_uOutputShadowReg.shadow_reg16 &= (LED_BLUE & LED_GREEN);
			break;

		case ledMagenta:
			m_uOutputShadowReg.shadow_reg16 &= (LED_BLUE & LED_RED);
			break;

		case ledYellow:
			m_uOutputShadowReg.shadow_reg16 &= (LED_RED & LED_GREEN);
			break;

		case ledWhite:
			m_uOutputShadowReg.shadow_reg16 &= (LED_BLUE & LED_GREEN & LED_RED);
			break;

		default:
			return 0;
	}

	write_reg16(OUTPUT_PORT0_REG, m_uOutputShadowReg.shadow_reg16);
	return 1;
}

uint8_t gpio_expander_init(void)
{
	m_uOutputShadowReg.shadow_reg16 = ~(LED_BLUE & LED_GREEN & LED_RED);

	write_reg16(OUTPUT_PORT0_REG, m_uOutputShadowReg.shadow_reg16);
	write_reg16(CONFIG_PORT0_REG, 0x1FFF);
#if 0
	write_reg8(OUTPUT_PORT1_REG, 0x1F);	//white
	write_reg8(OUTPUT_PORT1_REG, 0x3F);	//cyan
	write_reg8(OUTPUT_PORT1_REG, 0x5F);	//magenta
	write_reg8(OUTPUT_PORT1_REG, 0x7F);	//blue
	write_reg8(OUTPUT_PORT1_REG, 0x9F);	//yellow
	write_reg8(OUTPUT_PORT1_REG, 0xBF);	//green
	write_reg8(OUTPUT_PORT1_REG, 0xDF); //red
	write_reg8(OUTPUT_PORT1_REG, 0xFF);	//off
#endif //0
	gpio_expander_set_led_color(ledWhite);
	OSA_TimeDelay(100);
	gpio_expander_set_led_color(ledRed);
	OSA_TimeDelay(100);
	gpio_expander_set_led_color(ledYellow);
	OSA_TimeDelay(100);
	gpio_expander_set_led_color(ledBlue);
	OSA_TimeDelay(100);
	gpio_expander_set_led_color(ledMagenta);
	OSA_TimeDelay(100);
	gpio_expander_set_led_color(ledGreen);
	OSA_TimeDelay(100);
	gpio_expander_set_led_color(ledCyan);
	OSA_TimeDelay(100);
	gpio_expander_set_led_color(ledOff);
}

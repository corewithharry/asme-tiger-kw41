//////////////////////////////////////////////////////////////
// Copyright(c) 2016, Volansys Technologies
//
// Description:
/// \file app_accelerometer.c
/// \brief This is a file for the application accelerometer data reading.
///
//
// Author Volansys Technologies
//
//////////////////////////////////////////////////////////////

/*==================================================================================================
Include Files
==================================================================================================*/
/*  SDK Included Files */
#include "app_accelerometer.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_i2c.h"

#include "pin_mux.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define ACCEL_I2C_CLK_SRC I2C1_CLK_SRC
#define ACCEL_I2C_CLK_FREQ CLOCK_GetFreq(I2C1_CLK_SRC)

#define I2C_RELEASE_SDA_PORT PORTC
#define I2C_RELEASE_SCL_PORT PORTC
#define I2C_RELEASE_SDA_GPIO GPIOC
#define I2C_RELEASE_SDA_PIN 3U
#define I2C_RELEASE_SCL_GPIO GPIOC
#define I2C_RELEASE_SCL_PIN 2U
#define I2C_RELEASE_BUS_COUNT 100U
#define I2C_BAUDRATE 100000U
#define FXOS8700_WHOAMI 0xC7U
#define FXOS8700_WHOAMI_VARIANT	0xCAU
#define MMA8451_WHOAMI 0x1AU
#define ACCEL_STATUS 0x00U
#define ACCEL_XYZ_DATA_CFG 0x0EU
#define ACCEL_CTRL_REG1 0x2AU
#define ACCEL_M_CTRL_REG1 0x5B
#define ACCEL_M_CTRL_REG2 0x5C

/* FXOS8700 and MMA8451 have the same who_am_i register address. */
#define ACCEL_WHOAMI_REG 0x0DU
#define ACCEL_READ_TIMES 1U
#define ACC_BUFF_SIZE 256U

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void BOARD_I2C_ReleaseBus(void);

static bool I2C_ReadAccelWhoAmI(void);
static bool I2C_WriteAccelReg(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t value);
static bool I2C_ReadAccelRegs(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize);

//static void App_GetAccDataString(char *data);
/*******************************************************************************
 * Variables
 ******************************************************************************/

/*  FXOS8700 and MMA8451 device address */
const uint8_t g_accel_address[] = {0x1CU, 0x1DU, 0x1EU, 0x1FU};

i2c_master_handle_t g_m_handle;

uint8_t g_accel_addr_found = 0x00;

volatile bool completionFlag = false;
volatile bool nakFlag = false;

static bool isThereAccel = false;
int16_t xRawAccelData=0, yRawAccelData=0, zRawAccelData=0;
int16_t xRawMagData=0, yRawMagData=0, zRawMagData=0;

static float bit_res;

/*******************************************************************************
 * Code
 ******************************************************************************/
#if 0
static void i2c_release_bus_delay(void)
{
    uint32_t i = 0;
    for (i = 0; i < I2C_RELEASE_BUS_COUNT; i++)
    {
        __NOP();
    }
}

void BOARD_I2C_ReleaseBus(void)
{
    uint8_t i = 0;
    gpio_pin_config_t pin_config;
    port_pin_config_t i2c_pin_config = {0};

    /* Config pin mux as gpio */
    i2c_pin_config.pullSelect = kPORT_PullUp;
    i2c_pin_config.mux = kPORT_MuxAsGpio;

    pin_config.pinDirection = kGPIO_DigitalOutput;
    pin_config.outputLogic = 1U;
    CLOCK_EnableClock(kCLOCK_PortC);
    PORT_SetPinConfig(I2C_RELEASE_SCL_PORT, I2C_RELEASE_SCL_PIN, &i2c_pin_config);
    PORT_SetPinConfig(I2C_RELEASE_SCL_PORT, I2C_RELEASE_SDA_PIN, &i2c_pin_config);

    GPIO_PinInit(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, &pin_config);
    GPIO_PinInit(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, &pin_config);

    /* Drive SDA low first to simulate a start */
    GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
    i2c_release_bus_delay();

    /* Send 9 pulses on SCL and keep SDA low */
    for (i = 0; i < 9; i++)
    {
        GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
        i2c_release_bus_delay();

        GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
        i2c_release_bus_delay();

        GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
        i2c_release_bus_delay();
        i2c_release_bus_delay();
    }

    /* Send stop */
    GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
    i2c_release_bus_delay();

    GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
    i2c_release_bus_delay();
}
#endif //0

static bool I2C_ReadAccelWhoAmI(void)
{
    /*
    How to read the device who_am_I value ?
    Start + Device_address_Write , who_am_I_register;
    Repeart_Start + Device_address_Read , who_am_I_value.
    */
    uint8_t who_am_i_reg = ACCEL_WHOAMI_REG;
    uint8_t who_am_i_value = 0x00;
    uint8_t accel_addr_array_size = 0x00;
    bool find_device = false;
    uint8_t i = 0;
    //uint32_t sourceClock = 0;
    i2c_master_transfer_t masterXfer;
    char sendAccelerometerData[ACC_BUFF_SIZE];
    //i2c_master_config_t masterConfig;

    /*
     * masterConfig.baudRate_Bps = 100000U;
     * masterConfig.enableStopHold = false;
     * masterConfig.glitchFilterWidth = 0U;
     * masterConfig.enableMaster = true;
     */
    //I2C_MasterGetDefaultConfig(&masterConfig);

    //masterConfig.baudRate_Bps = I2C_BAUDRATE;

    //sourceClock = ACCEL_I2C_CLK_FREQ;

    //I2C_MasterInit(BOARD_ACCEL_I2C_BASEADDR, &masterConfig, sourceClock);

    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress = g_accel_address[0];
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data = &who_am_i_reg;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferNoStopFlag;

    accel_addr_array_size = sizeof(g_accel_address) / sizeof(g_accel_address[0]);

    for (i = 0; i < accel_addr_array_size; i++)
    {
        masterXfer.slaveAddress = g_accel_address[i];

        if(I2C_MasterTransferBlocking(BOARD_ACCEL_I2C_BASEADDR, &masterXfer) == kStatus_Success)
        {
            completionFlag = true;
        }
        else
        {
            PRINTF("Failed to transfer signals via I2C\r\n");
        }

        if (completionFlag == true)
        {
            completionFlag = false;
            find_device = true;
            g_accel_addr_found = masterXfer.slaveAddress;
            break;
        }
    }

    if (find_device == true)
    {
        masterXfer.direction = kI2C_Read;
        masterXfer.subaddress = 0;
        masterXfer.subaddressSize = 0;
        masterXfer.data = &who_am_i_value;
        masterXfer.dataSize = 1;
        masterXfer.flags = kI2C_TransferRepeatedStartFlag;
        
        if(I2C_MasterTransferBlocking(BOARD_ACCEL_I2C_BASEADDR, &masterXfer) == kStatus_Success)
        {
            completionFlag = true;
        }
        else
        {
            PRINTF("Failed to transfer signals via I2C\r\n");
        }

        if (completionFlag == true)
        {
            completionFlag = false;
            if (who_am_i_value == FXOS8700_WHOAMI || who_am_i_value == FXOS8700_WHOAMI_VARIANT)
            {
                PRINTF("Found an FXOS8700 on board , the device address is 0x%x . \r\n", masterXfer.slaveAddress);
                return true;
            }
            else if (who_am_i_value == MMA8451_WHOAMI)
            {
                PRINTF("Found an MMA8451 on board , the device address is 0x%x . \r\n", masterXfer.slaveAddress);
                return true;
            }
            else
            {
                PRINTF("Found a device, the WhoAmI value is 0x%x\r\n", who_am_i_value);
                PRINTF("It's not MMA8451 or FXOS8700. \r\n");
                PRINTF("The device address is 0x%x. \r\n", masterXfer.slaveAddress);

                memset(sendAccelerometerData,0,ACC_BUFF_SIZE);
                App_GetAccDataString(&sendAccelerometerData[0]);

                return false;
            }
        }
        else
        {
            PRINTF("Not a successful i2c communication \r\n");
            return false;
        }
    }
    else
    {
        PRINTF("\r\n Do not find an accelerometer device ! \r\n");
        return false;
    }
}

static bool I2C_WriteAccelReg(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t value)
{
    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress = device_addr;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = reg_addr;
    masterXfer.subaddressSize = 1;
    masterXfer.data = &value;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    if(I2C_MasterTransferBlocking(BOARD_ACCEL_I2C_BASEADDR, &masterXfer) == kStatus_Success)
    {
      completionFlag = true;
    }
    else
    {
      PRINTF("Failed to transfer signals via I2C\r\n");
    }

    if (completionFlag == true)
    {
        completionFlag = false;
        return true;
    }
    else
    {
        return false;
    }
}

static bool I2C_ReadAccelRegs(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize)
{
    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress = device_addr;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = reg_addr;
    masterXfer.subaddressSize = 1;
    masterXfer.data = rxBuff;
    masterXfer.dataSize = rxSize;
    masterXfer.flags = kI2C_TransferDefaultFlag;
    
    if(I2C_MasterTransferBlocking(BOARD_ACCEL_I2C_BASEADDR, &masterXfer) == kStatus_Success)
    {
      completionFlag = true;
    }
    else
    {
      PRINTF("Failed to transfer signals via I2C\r\n");
    }

    if (completionFlag == true)
    {
        completionFlag = false;
        return true;
    }
    else
    {
        return false;
    }
}


void accelerometerInit()
{
  uint8_t databyte = 0;
  uint8_t write_reg = 0;
  uint8_t readBuff[7];
  uint8_t status0_value = 0;
  
  isThereAccel = I2C_ReadAccelWhoAmI();
  if (true != isThereAccel)
  {
    shell_printf(" Accelerometer not found on the board\r\n");
  }

  /*  please refer to the "example FXOS8700CQ Driver Code" in FXOS8700 datasheet. */
  /*  write 0000 0000 = 0x00 to accelerometer control register 1 */
  /*  standby */
  /*  [7-1] = 0000 000 */
  /*  [0]: active=0 */
  write_reg = ACCEL_CTRL_REG1;
  databyte = 0;
  I2C_WriteAccelReg(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found, write_reg, databyte);

  // write 0001 1111 = 0x1F to magnetometer control register 1
  // [7]: m_acal=0: auto calibration disabled
  // [6]: m_rst=0: no one-shot magnetic reset
  // [5]: m_ost=0: no one-shot magnetic measurement
  // [4-2]: m_os=111=7: 8x oversampling (for 200Hz) to reduce magnetometer noise
  // [1-0]: m_hms=11=3: select hybrid mode with accel and magnetometer active
  write_reg = ACCEL_M_CTRL_REG1;
  databyte = 0x1F;
  I2C_WriteAccelReg(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found, write_reg, databyte);

  // write 0010 0000 = 0x20 to magnetometer control register 2
  // [7]: reserved
  // [6]: reserved
  // [5]: hyb_autoinc_mode=1 to map the magnetometer registers to follow the accelerometer registers
  // [4]: m_maxmin_dis=0 to retain default min/max latching even though not used
  // [3]: m_maxmin_dis_ths=0
  // [2]: m_maxmin_rst=0
  // [1-0]: m_rst_cnt=00 to enable magnetic reset each cycle
  write_reg = ACCEL_M_CTRL_REG2;
  databyte = 0x20;
  I2C_WriteAccelReg(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found, write_reg, databyte);

  /*  write 0000 0001= 0x01 to XYZ_DATA_CFG register */
  /*  [7]: reserved */
  /*  [6]: reserved */
  /*  [5]: reserved */
  /*  [4]: hpf_out=0 */
  /*  [3]: reserved */
  /*  [2]: reserved */
  /*  [1-0]: fs=01 for accelerometer range of +/-4g range with 0.488mg/LSB */
  /*  databyte = 0x01; */
  write_reg = ACCEL_XYZ_DATA_CFG;
  databyte = 0x01;
  I2C_WriteAccelReg(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found, write_reg, databyte);
  bit_res = 0.000488;	// bit resolution [g/LSB]

  /*  write 0000 1101 = 0x0D to accelerometer control register 1 */
  /*  [7-6]: aslp_rate=00 */
  /*  [5-3]: dr=001 for 200Hz data rate (when in hybrid mode) */
  /*  [2]: lnoise=1 for low noise mode */
  /*  [1]: f_read=0 for normal 16 bit reads */
  /*  [0]: active=1 to take the part out of standby and enable sampling */
  /*   databyte = 0x0D; */
  write_reg = ACCEL_CTRL_REG1;
  databyte = 0x0d;
  I2C_WriteAccelReg(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found, write_reg, databyte);
}

#if 1
/*!*************************************************************************************************
\private
\fn     void Accelerometer_CoapCb(bool_t sessionStatus, void *pData, coapSession_t *pSession, uint32_t dataLen)
\brief  This function is the callback function for CoAP message. 

\param  [in]    sessionStatus   status for CoAP session
\param  [in]    pData           pointer to CoAP message payload
\param  [in]    pSession        pointer to CoAP session
\param  [in]    dataLen         length of CoAP payload

\return         void
***************************************************************************************************/
void Accelerometer_CoapCb
(
    coapSessionStatus_t sessionStatus,
    void *pData,
    coapSession_t *pSession,
    uint32_t dataLen
)
{
  char sendAccelerometerData[ACC_BUFF_SIZE];
  char *pTempString = NULL;
  uint32_t ackPloadSize = 0;
  
  if(gCoapGET_c == pSession->code)
  {
    memset(sendAccelerometerData,0,ACC_BUFF_SIZE);
    App_GetAccDataString(&sendAccelerometerData[0]);
    pTempString = &sendAccelerometerData[0];
    ackPloadSize = strlen((char*)pTempString);
  }

  if (gCoapConfirmable_c == pSession->msgType)
  {
      if (gCoapGET_c == pSession->code)
      {
          COAP_Send(pSession, gCoapMsgTypeAckSuccessChanged_c, pTempString, ackPloadSize);
      }
      else
      {
          COAP_Send(pSession, gCoapMsgTypeAckSuccessChanged_c, NULL, 0);
      }
  }
}
#endif //0

void App_GetAccDataString(char *data)
{
    uint8_t databyte = 0;
    uint8_t write_reg = 0;
    uint8_t readBuff[13];
    uint8_t status0_value = 0;

    uint8_t cRetry;

    if (true == isThereAccel)
    {
#if 0
        /*  please refer to the "example FXOS8700CQ Driver Code" in FXOS8700 datasheet. */
        /*  write 0000 0000 = 0x00 to accelerometer control register 1 */
        /*  standby */
        /*  [7-1] = 0000 000 */
        /*  [0]: active=0 */
        write_reg = ACCEL_CTRL_REG1;
        databyte = 0;
        I2C_WriteAccelReg(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found, write_reg, databyte);

        // write 0001 1111 = 0x1F to magnetometer control register 1
        // [7]: m_acal=0: auto calibration disabled
        // [6]: m_rst=0: no one-shot magnetic reset
        // [5]: m_ost=0: no one-shot magnetic measurement
        // [4-2]: m_os=111=7: 8x oversampling (for 200Hz) to reduce magnetometer noise
        // [1-0]: m_hms=11=3: select hybrid mode with accel and magnetometer active
        write_reg = ACCEL_M_CTRL_REG1;
        databyte = 0x1F;
        I2C_WriteAccelReg(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found, write_reg, databyte);

        // write 0010 0000 = 0x20 to magnetometer control register 2
        // [7]: reserved
        // [6]: reserved
        // [5]: hyb_autoinc_mode=1 to map the magnetometer registers to follow the accelerometer registers
        // [4]: m_maxmin_dis=0 to retain default min/max latching even though not used
        // [3]: m_maxmin_dis_ths=0
        // [2]: m_maxmin_rst=0
        // [1-0]: m_rst_cnt=00 to enable magnetic reset each cycle
        write_reg = ACCEL_M_CTRL_REG2;
        databyte = 0x20;
        I2C_WriteAccelReg(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found, write_reg, databyte);

        /*  write 0000 0001= 0x01 to XYZ_DATA_CFG register */
        /*  [7]: reserved */
        /*  [6]: reserved */
        /*  [5]: reserved */
        /*  [4]: hpf_out=0 */
        /*  [3]: reserved */
        /*  [2]: reserved */
        /*  [1-0]: fs=01 for accelerometer range of +/-4g range with 0.488mg/LSB */
        /*  databyte = 0x01; */
        write_reg = ACCEL_XYZ_DATA_CFG;
        databyte = 0x01;
        I2C_WriteAccelReg(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found, write_reg, databyte);

        /*  write 0000 1101 = 0x0D to accelerometer control register 1 */
        /*  [7-6]: aslp_rate=00 */
        /*  [5-3]: dr=001 for 200Hz data rate (when in hybrid mode) */
        /*  [2]: lnoise=1 for low noise mode */
        /*  [1]: f_read=0 for normal 16 bit reads */
        /*  [0]: active=1 to take the part out of standby and enable sampling */
        /*   databyte = 0x0D; */
        write_reg = ACCEL_CTRL_REG1;
        databyte = 0x0d;
        I2C_WriteAccelReg(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found, write_reg, databyte);
#endif //0
        
        //OSA_TimeDelay(25);
        
        status0_value = 0;
        cRetry = 0;
        /*  wait for new data are ready. */
        while (status0_value != 0xff && cRetry < 20)
        {
            I2C_ReadAccelRegs(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found, ACCEL_STATUS, &status0_value, 1);
            cRetry++;
        }

        /*  Multiple-byte Read from STATUS (0x00) register */
        I2C_ReadAccelRegs(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found, ACCEL_STATUS, readBuff, 13);

        status0_value = readBuff[0];
    
        // Get the X and Y data of accelerometer from the sensor data structure.
        xRawAccelData = ((int16_t)(((readBuff[1] * 256U) | readBuff[2]))) / 4U;
        yRawAccelData = ((int16_t)(((readBuff[3] * 256U) | readBuff[4]))) / 4U;
        zRawAccelData = ((int16_t)(((readBuff[5] * 256U) | readBuff[6]))) / 4U;
        
        //shell_printf("Accelerometer readings are , x = %5d , y = %5d , z = %5d \r\n", xRawAccelData, yRawAccelData, zRawAccelData);
        xRawMagData = (((int16_t)(((readBuff[7] * 256U) | readBuff[8])))+5)/10;
        yRawMagData = (((int16_t)(((readBuff[9] * 256U) | readBuff[10])))+5)/10;
        zRawMagData = (((int16_t)(((readBuff[11] * 256U) | readBuff[12])))+5)/10;
        /* Compute output */
        sprintf((char*)data, "\"Ax\":\"%.3f\",\"Ay\":\"%.3f\",\"Az\":\"%.3f\",\"Mx\":\"%d\",\"My\":\"%d\",\"Mz\":\"%d\"", xRawAccelData*bit_res, yRawAccelData*bit_res, zRawAccelData*bit_res, xRawMagData, yRawMagData, zRawMagData);
    }
}

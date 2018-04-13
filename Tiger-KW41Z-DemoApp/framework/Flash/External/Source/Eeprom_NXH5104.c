/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* Copyright 2016-2017 NXP
*
* \file
*
* This is a source file which implements the driver for the AT45DB021E memory.
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
* o Neither the name of Freescale Semiconductor, Inc. nor the names of its
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


/*! *********************************************************************************
*************************************************************************************
* Include
*************************************************************************************
********************************************************************************** */
#include "EmbeddedTypes.h"
#include "Eeprom.h"

#if gEepromType_d == gEepromDevice_NXH5104_c

#include "SPI_Adapter.h"
#include "GPIO_Adapter.h"

/*! *********************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
********************************************************************************** */
#define  gEepromWriteEnable_d   1

#define  EEPROM_READ            0x03
#define  EEPROM_RDSR            0x05
#define  EEPROM_WREN			0x06

#define  EEPROM_WRITE_BYTES     0x02

#define  EEPROM_BUSY_FLAG_MASK  0x01
#define  EEPROM_PAGE_SIZE       (256)
#define  EEPROM_BLOCK_SIZE      (8*EEPROM_PAGE_SIZE)
#define  EEPROM_PAGE_MASK       (EEPROM_PAGE_SIZE - 1)

/* adress mask */
#define  ADDRESS_MASK 0x000000FF


/* SPI config */
#define gEepromSpiInstance_c  0

#define gEepromAssertCS_d()   GpioClearPinOutput(&mEepromSpiCsCfg)
#define gEepromDeassertCS_d() GpioSetPinOutput(&mEepromSpiCsCfg)


/******************************************************************************
*******************************************************************************
* Private Prototypes
*******************************************************************************
******************************************************************************/
#if gEepromWriteEnable_d
static void EEPROM_WriteEnable(void);
static ee_err_t EEPROM_WritePage(uint32_t NoOfBytes, uint32_t Addr, uint8_t *Outbuf);
#endif
static ee_err_t EEPROM_WaitForReady(void);
static uint16_t EEPROM_ReadStatusReq(void);
static void     EEPROM_Command(uint8_t opCode, uint32_t Addr);


/*! *********************************************************************************
*************************************************************************************
* Private Memory Declarations
*************************************************************************************
********************************************************************************** */
static spiState_t mEepromSpiState;
const gpioOutputPinConfig_t mEepromSpiCsCfg = {
    .gpioPort = gpioPort_C_c,
    .gpioPin = 19,
    .outputLogic = 1,
    .slewRate = pinSlewRate_Fast_c,
    .driveStrength = pinDriveStrength_Low_c
};

/*! *********************************************************************************
*************************************************************************************
* Public Functions
*************************************************************************************
********************************************************************************** */

/*****************************************************************************
*  EEPROM_Init
*
*  Initializes the EEPROM peripheral
*
*****************************************************************************/
ee_err_t EEPROM_Init(void)
{
    static uint8_t initialized = 0;
    ee_err_t retval;
    //const uint8_t cmd[] = {0x3D, 0x2A, 0x80, 0xA6};
    spiBusConfig_t spiConfig = {
        .bitsPerSec = 5000000,
        .master = TRUE,
        .clkActiveHigh = TRUE,
        .clkPhaseFirstEdge = TRUE,
        .MsbFirst = TRUE
    };


    if( !initialized )
    {
        Spi_Init(gEepromSpiInstance_c, &mEepromSpiState, NULL, NULL);
        Spi_Configure(gEepromSpiInstance_c, &spiConfig);
        GpioOutputPinInit(&mEepromSpiCsCfg, 1);

        retval = EEPROM_WaitForReady();
        if (retval != ee_ok)
            return retval;

        /* Set page size to 256bits: */
        //gEepromAssertCS_d();
        //Spi_SyncTransfer(gEepromSpiInstance_c, (uint8_t*)cmd, NULL, sizeof(cmd));
        //gEepromDeassertCS_d();
        initialized = 1;
    }

    return ee_ok;
}


/*****************************************************************************
*  EEPROM_WriteData
*
*  Writes a data buffer into EEPROM, at a given address
*
*****************************************************************************/
#if gEepromWriteEnable_d
ee_err_t EEPROM_WriteData(uint32_t NoOfBytes, uint32_t Addr, uint8_t *Outbuf)
{
    ee_err_t retval;

    if (NoOfBytes == 0)
        return ee_ok;

    while (EEPROM_isBusy());

    while ((Addr & EEPROM_PAGE_MASK) + NoOfBytes > EEPROM_PAGE_MASK)
    {
        uint32_t bytes = EEPROM_PAGE_SIZE - (Addr & EEPROM_PAGE_MASK);

        EEPROM_WriteEnable();
        retval = EEPROM_WritePage(bytes, Addr, Outbuf);
        NoOfBytes -= bytes;
        Addr += bytes;
        Outbuf += bytes;

        if (retval != ee_ok)
            return retval;
    }

    EEPROM_WriteEnable();
    retval = EEPROM_WritePage(NoOfBytes, Addr, Outbuf);

    return retval;
}
#endif

/*****************************************************************************
*  EEPROM_ReadData
*
*  Reads a data buffer from EEPROM, from a given address
*
*****************************************************************************/
ee_err_t EEPROM_ReadData(uint16_t NoOfBytes, uint32_t Addr, uint8_t *inbuf)
{

    while (EEPROM_isBusy());

    EEPROM_Command(EEPROM_READ,Addr);

    Spi_SyncTransfer(gEepromSpiInstance_c, NULL, inbuf, NoOfBytes);
    gEepromDeassertCS_d();

    return ee_ok;
}

/*****************************************************************************
*  EEPROM_ReadStatusReq
*
*
*****************************************************************************/
uint8_t EEPROM_isBusy(void)
{
    return (EEPROM_ReadStatusReq() & EEPROM_BUSY_FLAG_MASK);
}

/*! *********************************************************************************
*************************************************************************************
* Private Functions
*************************************************************************************
********************************************************************************** */

/*****************************************************************************
*  EEPROM_ReadStatusReq
*
*
*****************************************************************************/
static uint16_t EEPROM_ReadStatusReq(void)
{
    const uint8_t cmd = EEPROM_RDSR;
    uint8_t data = 0;

    gEepromAssertCS_d();
    Spi_SyncTransfer(gEepromSpiInstance_c, (uint8_t*)&cmd, NULL, sizeof(cmd));
    Spi_SyncTransfer(gEepromSpiInstance_c, NULL, (uint8_t*)&data, 1);
    gEepromDeassertCS_d();

    return data;
}

static void EEPROM_WriteEnable(void)
{
    const uint8_t cmd = EEPROM_WREN;

    gEepromAssertCS_d();
    Spi_SyncTransfer(gEepromSpiInstance_c, (uint8_t*)&cmd, NULL, sizeof(cmd));
    gEepromDeassertCS_d();
}

/*****************************************************************************
*  EEPROM_WritePage
*
*  Writes maximum 256 bytes into a EEPROM page
*
*****************************************************************************/
#if gEepromWriteEnable_d
static ee_err_t EEPROM_WritePage(uint32_t NoOfBytes, uint32_t Addr, uint8_t *Outbuf)
{

    if (NoOfBytes == 0)
        return ee_ok;

    while (EEPROM_isBusy());

    /*SPI_Send_byte(EEPROM_WRITE_ENABLE,DEASSERT_CS) */

    EEPROM_Command(EEPROM_WRITE_BYTES, Addr); /*CS will remain ASSERTED */

    Spi_SyncTransfer(gEepromSpiInstance_c, Outbuf, NULL, NoOfBytes);
    gEepromDeassertCS_d();

    return ee_ok;
}
#endif

/*****************************************************************************
*  EEPROM_WaitForReady
*
*
*****************************************************************************/
static ee_err_t EEPROM_WaitForReady(void)
{
    volatile uint16_t wait = 0x400; /* near 50 ms @ 50 us/cycle */

    /* Byte1 - Bit:   7       6      5:2       1         0
    *             RDY/BUSY  COMP  DENSITY  PROTECT  PAGE_SIZE
    *
    * Byte2 - Bit:   7       6    5    4    3    2    1    0
    *             RDY/BUSY  RES  EPE  RES  SLE  PS2  PS1  ES
    */

    while( EEPROM_isBusy() && (wait !=0) )
    {
        wait--;
    }

    if(wait != 0)
    {
        return ee_ok;
    }
    return ee_error;
}


/*****************************************************************************
*  EEPROM_Command
*
*
*****************************************************************************/
static void EEPROM_Command(uint8_t opCode, uint32_t Addr)
{
    uint8_t cmd[4];

    cmd[0] = opCode;
    cmd[1] = (Addr >> 16) & ADDRESS_MASK;
    cmd[2] = (Addr >>  8) & ADDRESS_MASK;
    cmd[3] = (Addr >>  0) & ADDRESS_MASK;

    gEepromAssertCS_d();
    Spi_SyncTransfer(gEepromSpiInstance_c, cmd, NULL, sizeof(cmd));
    /* CS will remain asserted to be able to send the data */
}

#endif
/* EOF: Eeprom_AT45DB041E */
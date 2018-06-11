/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/** ============================================================================
 *  @file       sensortag_i2c.c
 *
 *  @brief      Simple interface to the TI-RTOS driver. Also manages switching
 *              between I2C-buses.
 *
 *  ============================================================================
 */

/* -----------------------------------------------------------------------------
*  Includes
* ------------------------------------------------------------------------------
*/
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ICall.h>
#include <ti/drivers/I2C.h>
#include "Board.h"
#include "myi2c.h"
/* -----------------------------------------------------------------------------
*  Constants
* ------------------------------------------------------------------------------
*/
#define I2C_TIMEOUT 500
#define MS_2_TICKS(ms)   (((ms) * 1000) / Clock_tickPeriod)
/* -----------------------------------------------------------------------------
*  Public Variables
* ------------------------------------------------------------------------------
*/

/* -----------------------------------------------------------------------------
*  Private Variables
* ------------------------------------------------------------------------------
*/

/* I2C driver interface */
static I2C_Handle i2cHandle;
static I2C_Params i2cParams;
static Semaphore_Struct mutex;
/* Module state */

//static volatile uint8_t slaveAddr = 0x68;
static uint8_t buffer[32];

/* -----------------------------------------------------------------------------
*  Public Functions
* ------------------------------------------------------------------------------
*/

/*******************************************************************************
* @fn          SensorI2C_write
*
* @brief       Burst write to an I2C device
*
* @param       data - pointer to data buffer
* @param       len - number of bytes to write
*
* @return      true if success
*/
bool SensorI2C_write(uint8_t slaveaddr, uint8_t *data, uint8_t len)
{
    I2C_Transaction masterTransaction;

    masterTransaction.writeCount   = len;
    masterTransaction.writeBuf     = data;
    masterTransaction.readCount    = 0;
    masterTransaction.readBuf      = NULL;
    masterTransaction.slaveAddress = slaveaddr;

    return I2C_transfer(i2cHandle, &masterTransaction) == TRUE;
}

/*******************************************************************************
* @fn          SensorI2C_writeRead
*
* @brief       Burst write/read from an I2C device
*
* @param       wdata - pointer to write data buffer
* @param       wlen - number of bytes to write
* @param       rdata - pointer to read data buffer
* @param       rlen - number of bytes to read
*
* @return      true if success
*/
bool SensorI2C_writeRead(uint8_t slaveaddr, uint8_t *wdata, uint8_t wlen, uint8_t *rdata, uint8_t rlen)
{
    I2C_Transaction masterTransaction;

    masterTransaction.writeCount = wlen;
    masterTransaction.writeBuf = wdata;
    masterTransaction.readCount = rlen;
    masterTransaction.readBuf = rdata;
    masterTransaction.slaveAddress = slaveaddr;

    return I2C_transfer(i2cHandle, &masterTransaction) == TRUE;
}

/*******************************************************************************
* @fn          SensorI2C_readReg
*
* @brief       This function implements the I2C protocol to read from a sensor.
*              The sensor must be selected before this routine is called.
*
* @param       addr - which register to read
* @param       pBuf - pointer to buffer to place data
* @param       nBytes - number of bytes to read
*
* @return      TRUE if the required number of bytes are received
******************************************************************************/
bool SensorI2C_readReg(uint8_t slaveaddr, uint8_t addr, uint8_t *pBuf, uint8_t nBytes)
{
    Semaphore_pend(Semaphore_handle(&mutex),MS_2_TICKS(I2C_TIMEOUT));

    uint8_t success = 0;
    success = SensorI2C_writeRead(slaveaddr, &addr, 1, pBuf, nBytes);
    Semaphore_post(Semaphore_handle(&mutex));
    return success;
}

/*******************************************************************************
* @fn          SensorI2C_writeReg
* @brief       This function implements the I2C protocol to write to a sensor.
*              The sensor must be selected before this routine is called.
*
* @param       addr - which register to write
* @param       pBuf - pointer to buffer containing data to be written
* @param       nBytes - number of bytes to write
*
* @return      TRUE if successful write
*/
bool SensorI2C_writeReg(uint8_t slaveaddr, uint8_t addr, uint8_t pBuf)
{
/*
    uint8_t i;
    uint8_t *p = buffer;
*/
    /* Copy address and data to local buffer for burst write */
/*
    *p++ = addr;
    for (i = 0; i < nBytes; i++)
    {
        *p++ = *pBuf++;
    }
    nBytes++;
*/
    Semaphore_pend(Semaphore_handle(&mutex),MS_2_TICKS(I2C_TIMEOUT));
    uint8_t *p = buffer;
    *p++ = addr;
    *p++ = pBuf++;

    /* Send data */
    uint8_t success = 0;
    success = SensorI2C_write(slaveaddr, buffer, 2);
    Semaphore_post(Semaphore_handle(&mutex));
    return success;
}

/*******************************************************************************
* @fn          SensorI2C_open
*
* @brief       Initialize the RTOS I2C driver (must be called only once)
*
* @param       none
*
* @return      true if I2C_open succeeds
*/
bool SensorI2C_open(void)
{
    Semaphore_Params semParamsMutex;

    // Create protection semaphore
    Semaphore_Params_init(&semParamsMutex);
    semParamsMutex.mode = Semaphore_Mode_BINARY;
    Semaphore_construct(&mutex, 1, &semParamsMutex);

    // Initialize I2C bus
    I2C_init();
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_100kHz;
    i2cHandle = I2C_open(Board_I2C, &i2cParams);


    return i2cHandle != NULL;
}

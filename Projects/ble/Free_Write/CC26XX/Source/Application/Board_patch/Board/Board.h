/*
 * Copyright (c) 2015, Texas Instruments Incorporated
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
 *  @file       Board.h
 *
 *  @brief      CC2650SENSORTAG Board Specific header file.
 *
 *  NB! This is the board file for PCB versions 1.2 and 1.3
 *
 *  The CC2650 header file should be included in an application as follows:
 *  @code
 *  #include <Board.h>
 *  @endcode
 *
 *  ============================================================================
 */
#ifndef __BOARD_H__
#define __BOARD_H__

#ifdef __cplusplus
extern "C" {
#endif

/** ============================================================================
 *  Includes
 *  ==========================================================================*/
#include <ti/drivers/PIN.h>
#include <driverlib/ioc.h>

/** ============================================================================
 *  Externs
 *  ==========================================================================*/
extern PIN_Config BoardGpioInitTable[];

/** ============================================================================
 *  Defines
 *  ==========================================================================*/

/* Symbol by generic Board.c to include the correct PCB  specific Board.c */
#define CC2650ST_0120
  
/* Identify as SensorTag */
#define CC2650ST_7ID

/* Same RF Configuration as 7x7 EM */
#define CC2650EM_7ID

/* This PCB version supports magnetometer */
#define FEATURE_MAGNETOMETER

/* External flash manufacturer and device ID */
#define EXT_FLASH_MAN_ID            0xEF
#define EXT_FLASH_DEV_ID            0x12

/* Mapping of pins to board signals using general board aliases
 *      <board signal alias>                <pin mapping>
 */

/* Discrete outputs */
#define Board_LED1                  IOID_1
#define Board_LED_ON                1
#define Board_LED_OFF               0

/* Discrete inputs */
#define Board_KEY                   IOID_0


/* I2C */
#define Board_I2C0_SDA0             IOID_8
#define Board_I2C0_SCL0             IOID_9
#define Board_I2C0_SDA1             IOID_2
#define Board_I2C0_SCL1             IOID_3

/* Power control */
#define Board_PER_POWER             IOID_4
#define Board_PER_POWER_ON          1
#define Board_PER_POWER_OFF         0
   
/* ADC */
#define Board_POWERDet              ADC_COMPB_IN_AUXIO7
#define Board_POWERDetDIO           IOID_7

/* UART pins used by driver */
#define Board_UART_TX               IOID_13
//#define Board_UART_TX               IOID_10
#define Board_UART_RX               IOID_14

/* User Defined Keys */
#define PeripheralKey1              IOID_10
//#define PeripheralKey1              IOID_5
#define PeripheralKeyPress          0

/** ============================================================================
 *  Instance identifiers
 *  ==========================================================================*/
/* Generic I2C instance identifiers */
#define Board_I2C                   CC2650_I2C0
/* Generic UART instance identifiers */
#define Board_UART                  CC2650_UART0
   
#define Board_ADC                   CC2650_ADC0


/** ============================================================================
 *  Number of peripherals and their names
 *  ==========================================================================*/

/*!
 *  @def    CC2650_I2CName
 *  @brief  Enum of I2C names on the CC2650 dev board
 */
typedef enum CC2650_I2CName {
    CC2650_I2C0 = 0,
    CC2650_I2CCOUNT
} CC2650_I2CName;

/*!
 *  @def    CC2650_CryptoName
 *  @brief  Enum of Crypto names on the CC2650 dev board
 */
typedef enum CC2650_CryptoName {
    CC2650_CRYPTO0 = 0,
    CC2650_CRYPTOCOUNT
} CC2650_CryptoName;


/*!
 *  @def    CC2650_SPIName
 *  @brief  Enum of SPI names on the CC2650 dev board
 */
typedef enum CC2650_SPIName {
    CC2650_SPICOUNT
} CC2650_SPIName;

/*!
 *  @def    CC2650_UARTName
 *  @brief  Enum of UARTs on the CC2650 dev board
 */
typedef enum CC2650_UARTName {
    CC2650_UART0 = 0,
    CC2650_UARTCOUNT
} CC2650_UARTName;

/*!
 *  @def    CC2650_ADCName
 *  @brief  Enum of UARTs on the CC2650 dev board
 */
typedef enum CC2650_ADCName {
    CC2650_ADC0 = 0,
    CC2650_ADCCOUNT
} CC2650_ADCName;

/*!
 *  @def    CC2650_UdmaName
 *  @brief  Enum of DMA buffers
 */
typedef enum CC2650_UdmaName {
    CC2650_UDMA0 = 0,
    CC2650_UDMACOUNT
} CC2650_UdmaName;

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H__ */

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

/*
 *  ====================== Board.c =============================================
 *  This file is responsible for setting up the board specific items for the
 *  CC2650 SensorTag. 
 *
 *  NB! This board file is for PCB version 1.2 and 1.3
 */


/*
 *  ====================== Includes ============================================
 */
#include <inc/hw_memmap.h>
#include <inc/hw_ints.h>
#include <driverlib/ioc.h>
#include <driverlib/udma.h>
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/sysbios/family/arm/cc26xx/Power.h>
#include <ti/sysbios/family/arm/cc26xx/PowerCC2650.h>
#include <ti/drivers/PIN.h>
#include "Board.h"

/*
 *  ========================= IO driver initialization =========================
 *  From main, PIN_init(BoardGpioInitTable) should be called to setup safe
 *  settings for this board.
 *  When a pin is allocated and then de-allocated, it will revert to the state
 *  configured in this table.
*/

PIN_Config BoardGpioInitTable[] = {
    Board_LED1       | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_OPENSOURCE | PIN_DRVSTR_MAX,    /* LED initially off             */
    Board_KEY        | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,         /* Button is active low          */
    Board_PER_POWER  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MED,    /* Peripheral Power initially off*/
    Board_POWERDetDIO   | PIN_INPUT_DIS | PIN_GPIO_OUTPUT_DIS,
    PeripheralKey1   | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,
    PIN_TERMINATE
};
/*============================================================================*/

/*
 *  ============================= UART begin ===================================
*/
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(UART_config, ".const:UART_config")
#pragma DATA_SECTION(uartCC26XXHWAttrs, ".const:uartCC26XXHWAttrs")
#endif

/* Include drivers */
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTCC26XX.h>

/* UART objects */
UARTCC26XX_Object uartCC26XXObjects[CC2650_UARTCOUNT];

/* UART hardware parameter structure, also used to assign UART pins */
const UARTCC26XX_HWAttrs uartCC26XXHWAttrs[CC2650_UARTCOUNT] = {
    {    /* CC2650_UART0 */
        .baseAddr = UART0_BASE,
        .powerMngrId = PERIPH_UART0,
        .intNum = INT_UART0,
        .txPin = Board_UART_TX,
        .rxPin = Board_UART_RX,
        .ctsPin = PIN_UNASSIGNED,
        .rtsPin = PIN_UNASSIGNED
    }
};

/* UART configuration structure */
const UART_Config UART_config[] = {
    { &UARTCC26XX_fxnTable, &uartCC26XXObjects[0], &uartCC26XXHWAttrs[0] },
    { NULL, NULL, NULL }
};
/*
 *  ============================= UART end =====================================
*/

/*
 *  ============================= UDMA begin ===================================
*/
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(UDMACC26XX_config, ".const:UDMACC26XX_config")
#pragma DATA_SECTION(udmaHWAttrs, ".const:udmaHWAttrs")
#endif

/* Include drivers */
#include <ti/drivers/dma/UDMACC26XX.h>

/* UDMA objects */
UDMACC26XX_Object UdmaObjects[CC2650_UDMACOUNT];

/* UDMA configuration structure */
const UDMACC26XX_HWAttrs udmaHWAttrs[CC2650_UDMACOUNT] = {
    {
      .baseAddr = UDMA0_BASE,
      .powerMngrId = PERIPH_UDMA,
      .intNum = INT_UDMAERR,
    }
};

/* UDMA configuration structure */
const UDMACC26XX_Config UDMACC26XX_config[] = {
    {&UdmaObjects[0], &udmaHWAttrs[0]},
    {NULL, NULL},
};
/*
 *  ============================= UDMA end =====================================
*/

/*
 *  ============================= I2C Begin=====================================
*/
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(I2C_config, ".const:I2C_config")
#pragma DATA_SECTION(i2cCC26xxHWAttrs, ".const:i2cCC26xxHWAttrs")
#endif

/* Include drivers */
#include <ti/drivers/i2c/I2CCC26XX.h>

/* I2C objects */
I2CCC26XX_Object i2cCC26xxObjects[CC2650_I2CCOUNT];

/* I2C configuration structure, describing which pins are to be used */
const I2CCC26XX_HWAttrs i2cCC26xxHWAttrs[CC2650_I2CCOUNT] = {
    {
        .baseAddr = I2C0_BASE,
        .powerMngrId = PERIPH_I2C0,
        .intNum = INT_I2C,
        .sdaPin = Board_I2C0_SDA0,
        .sclPin = Board_I2C0_SCL0,
    }
};

const I2C_Config I2C_config[] = {
    {&I2CCC26XX_fxnTable, &i2cCC26xxObjects[0], &i2cCC26xxHWAttrs[0]},
    {NULL, NULL, NULL}
};
/*
 *  ========================== I2C end =========================================
*/
/*
 *  ============================= ADC begin ===================================
*/

/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(ADC_config, ".const:ADC_config")
#pragma DATA_SECTION(adcCC26XX_HWAttrs, ".const:adcCC26XX_HWAttrs")
#endif

/* Include drivers */
#include <ti/drivers/adc/ADCCC26XX.h>
/* I2C objects */
ADCCC26XX_Object adcCC26xxObjects[CC2650_ADCCOUNT];

/* I2C configuration structure, describing which pins are to be used */
const ADCCC26XX_HWAttrs adcCC26XX_HWAttrs[CC2650_ADCCOUNT] = {
    {
      .adcDIO = Board_POWERDetDIO,
      .adcCompBInput = Board_POWERDet,
      .inputScalingEnabled = true,
      .refSource = ADCCC26XX_FIXED_REFERENCE,
      .samplingDuration = ADCCC26XX_SAMPLING_DURATION_2P7_US,
      .triggerSource = ADCCC26XX_TRIGGER_MANUAL
    }
};

const ADC_Config ADC_config[] = {
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[0], &adcCC26XX_HWAttrs[0]},
    {NULL, NULL, NULL}
};


/*
 *  ========================== ADC end =========================================
*/
/*
 *  ========================== Crypto begin =======================================
 *  NOTE: The Crypto implementation should be considered experimental and not validated!
*/
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(CryptoCC26XX_config, ".const:CryptoCC26XX_config")
#pragma DATA_SECTION(cryptoCC26XXHWAttrs, ".const:cryptoCC26XXHWAttrs")
#endif

/* Include drivers */
#include <ti/drivers/crypto/CryptoCC26XX.h>

/* Crypto objects */
CryptoCC26XX_Object cryptoCC26XXObjects[CC2650_CRYPTOCOUNT];

/* Crypto configuration structure, describing which pins are to be used */
const CryptoCC26XX_HWAttrs cryptoCC26XXHWAttrs[CC2650_CRYPTOCOUNT] = {
    {
      .baseAddr = CRYPTO_BASE,
      .powerMngrId = PERIPH_CRYPTO,
      .intNum = INT_CRYPTO,
    }
};

/* Crypto configuration structure */
const CryptoCC26XX_Config CryptoCC26XX_config[] = {
    {&cryptoCC26XXObjects[0], &cryptoCC26XXHWAttrs[0]},
    {NULL, NULL}
};

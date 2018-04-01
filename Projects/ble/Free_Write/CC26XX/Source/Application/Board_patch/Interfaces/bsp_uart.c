/*******************************************************************************
*  Filename:       bsp_uart.c
*  Revised:        $Date: $
*  Revision:       $Revision: $
*
*  Description:    Layer added on top of RTOS driver for backward
*                  compatibility with non RTOS UART driver.
*
*  Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*******************************************************************************/
#ifdef TI_DRIVERS_UART_INCLUDED

/*******************************************************************************
 * INCLUDES
 */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/family/arm/cc26xx/Power.h>

#include <ti/drivers/uart/UARTCC26XX.h>

#include <driverlib/prcm.h>

#include "Board.h"
#include "sensor.h"

#include "bsp_uart.h"

/*******************************************************************************
 * CONSTANTS
 */
#define UART_PRINTF_BUF_LEN      512
#define UART_TIMEOUT             2500

/*******************************************************************************
 * GLOBAL variables
 */

/*******************************************************************************
 * LOCAL variables
 */
static UART_Handle uartHandle;
static UART_Params uartParams;
static Semaphore_Struct mutex;
static uint8_t  uartPrintf_outArray[UART_PRINTF_BUF_LEN];
static uint16_t uartPrintf_head = 0;
static uint16_t uartPrintf_tail = 0;

/*******************************************************************************
 * @fn          bspUartDeselect
 *
 * @brief       Allow other tasks to access the UART driver
 *
 * @param       none
 *
 * @return      none
 */
void bspUartClose(void)
{
    // Close the driver
  UART_close(uartHandle);
}


/*******************************************************************************
 * @fn          bspUartInit
 *
 * @brief       Initialize the RTOS UART driver (must be called only once)
 *
 * @param       none
 *
 * @return      none
 */
void bspUartInit(void)
{

  Semaphore_Params semParamsMutex;

  // Create protection semaphore
  Semaphore_Params_init(&semParamsMutex);
  semParamsMutex.mode = Semaphore_Mode_BINARY;
  Semaphore_construct(&mutex, 1, &semParamsMutex);
  
  // Reset the UART controller
  HapiResetPeripheral(PRCM_PERIPH_UART0);

  UART_init();
  UART_Params_init(&uartParams);
  
  uartHandle = UART_open(Board_UART, &uartParams);
  
  if (uartHandle == NULL)
  {
    Task_exit();
  }
}

/*******************************************************************************
 * @fn          bspUartReset
 *
 * @brief       Reset the RTOS UART driver
 *
 * @param       none
 *
 * @return      none
 */
void bspUartReset(void)
{
  // Close the driver
  UART_close(uartHandle);

  // Reset the UART controller
  HapiResetPeripheral(PRCM_PERIPH_UART0);

  // Open driver
  uartHandle = UART_open(Board_UART, &uartParams);
}

/*********************************************************************
 * SYSTEM HOOK FUNCTIONS
 */

/*********************************************************************
 * @fn      uartPrintf_putch
 *
 * @brief   User supplied PutChar function.
 *          typedef Void (*SysCallback_PutchFxn)(Char);
 *
 *          This function is called whenever the System module needs
 *          to output a character.
 *
 *          This implementation fills a very basic ring-buffer, and relies
 *          on another function to flush this buffer out to UART.
 *
 *          Requires SysCallback to be the system provider module.
 *          Initialized via SysCallback.putchFxn = "&uartPrintf_putch"; in the
 *          TI-RTOS configuration script.
 *
 * @param   ch - Character
 *
 * @return  None.
 *
 * @post    ::uartPrintf_head is incremented by one with wrap at UART_PRINTF_BUF_LEN
 *          if there is room.
 */
void uartPrintf_putch(char ch)
{
    // uartPrintf_tail should never catch up with uartPrintf_head. Discard in-between bytes.
  if ( (uartPrintf_head + 1) % UART_PRINTF_BUF_LEN == uartPrintf_tail )
    return;
  // Acquire UART resource
  if (!Semaphore_pend(Semaphore_handle(&mutex),MS_2_TICKS(UART_TIMEOUT)))
  {
    return;
  }
  uartPrintf_outArray[uartPrintf_head] = ch;
  uartPrintf_head++;

  if (uartPrintf_head >= UART_PRINTF_BUF_LEN)
          uartPrintf_head = 0;
        
  // Release UART resource
  Semaphore_post(Semaphore_handle(&mutex));
}

/*********************************************************************
 * @fn      uartPrintf_flush
 *
 * @brief   Printf-buffer flush function
 *
 *          In this implementation it is intended to be called by the
 *          Idle task when nothing else is running.
 *
 *          This is achieved by setting up the Idle task in the TI-RTOS
 *          configuration script like so:
 *
 *          var Idle = xdc.useModule('ti.sysbios.knl.Idle');
 *          Idle.addFunc('&uartPrintf_flush');
 *
 * @param   None. Relies on global state.
 *
 * @return  None.
 *
 * @post    ::uartPrintf_tail is incremented to where uartPrintf_head
 *          was at the time the function was called.
  */
void uartPrintf_flush()
{
  // Abort in case UART hasn't been initialized.
  if (NULL == uartHandle)
    return;

  // Lock head position to avoid race conditions
  uint16_t curHead = uartPrintf_head;

  // Find out how much data must be output, and how to output it.
  bool needWrap = curHead < uartPrintf_tail;
  uint16_t outLen = needWrap?(UART_PRINTF_BUF_LEN-uartPrintf_tail+curHead):(curHead-uartPrintf_tail);

  if (outLen)
  {
    if (needWrap)
    {
      UART_write(uartHandle, &uartPrintf_outArray[uartPrintf_tail], UART_PRINTF_BUF_LEN - uartPrintf_tail);
      UART_write(uartHandle, uartPrintf_outArray, curHead);
    }
    else
    {
      UART_write(uartHandle, &uartPrintf_outArray[uartPrintf_tail], outLen);
    }
  }

  uartPrintf_tail = curHead;
}

#endif

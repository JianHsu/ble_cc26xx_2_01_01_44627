/*******************************************************************************
  Filename:       myuart.c

  Revised:
  Revision:

  Description:    send uart.
*
*/

#include <ti/sysbios/knl/Task.h>

#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/UART.h>
#include "Board.h"
#include "string.h"
#include "myuart.h"

#define UART_TASKSTACKSIZE     768

Task_Struct UarttaskStruct;
Char UarttaskStack[UART_TASKSTACKSIZE];

#define UART_PRINT_BUF_LEN      1024
uint8_t  uartPrint_outArray[UART_PRINT_BUF_LEN];
volatile uint16_t uartPrint_head = 0;
volatile uint16_t uartPrint_tail = 0;
/*********************************************************************
 * @fn      Uart_Print
 *
 * @brief   Uart Print Function.
 *
 * @param   uint8_t* str.
 *
 * @return  none
 */
void Uart_Print(uint8_t* str)
{
    uint16_t i = 0;

    for (i = 0; i < strlen(str); i++){
        uartPrint_tail++;
        if (uartPrint_tail >= UART_PRINT_BUF_LEN)
            uartPrint_tail = 0;

        uartPrint_outArray[uartPrint_tail] = str[i];
    }

}
/*********************************************************************
 * @fn      Uart_taskFxn
 *
 * @brief   Uart Application Task event processor.
 *
 * @param   a0, a1 - not used.
 *
 * @return  none
 */
Void UartFxn(UArg arg0, UArg arg1)
{
    UART_Handle uart;
    UART_Params uartParams;

    Board_initUART();

    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 115200;
    uart = UART_open(Board_UART0, &uartParams);

    if (uart == NULL) {
        PINCC26XX_setOutputValue(Board_RLED, 1);
    }

    /* Loop forever echoing */
    while (1) {
        if (uartPrint_head != uartPrint_tail){

            uartPrint_head++;
            if (uartPrint_head >= UART_PRINT_BUF_LEN)
                uartPrint_head = 0;

            UART_write(uart, &uartPrint_outArray[uartPrint_head], 1);
        }
    }
}

/*********************************************************************
 * @fn      Uart_createTask
 *
 * @brief   Task creation function for the HID emulated keyboard.
 *
 * @param   none
 *
 * @return  none
 */
void Uart_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = &UarttaskStack;
  taskParams.stackSize = UART_TASKSTACKSIZE;

  Task_construct(&UarttaskStruct, (Task_FuncPtr)UartFxn, &taskParams, NULL);
}

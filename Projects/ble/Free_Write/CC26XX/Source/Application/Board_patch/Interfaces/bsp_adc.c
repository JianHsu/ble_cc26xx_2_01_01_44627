#ifdef TI_DRIVERS_UART_INCLUDED

/*******************************************************************************
 * INCLUDES
 */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/family/arm/cc26xx/Power.h>

#include <ti/drivers/ADC.h>
#include <ti/drivers/adc/ADCCC26XX.h>

#include "Board.h"
#include "sensor.h"
#include "bsp_adc.h"

/*******************************************************************************
 * CONSTANTS
 */
#define ADC_TIMEOUT 2500
/*******************************************************************************
 * GLOBAL variables
 */

/*******************************************************************************
 * LOCAL variables
 */

static ADC_Handle adcHandle;
static ADC_Params adcParams;
static Semaphore_Struct mutex;

/*******************************************************************************
 * @fn          bspADCOpen
 *
 * @brief       acquire ADC source
 *
 * @param       none
 *
 * @return      none
 */
bool bspADCOpen(void)
{
  if (!Semaphore_pend(Semaphore_handle(&mutex),MS_2_TICKS(ADC_TIMEOUT)))
    return false;
  return true;
}
/*******************************************************************************
 * @fn          bspADCClose
 *
 * @brief       Allow other tasks to access the ADC driver
 *
 * @param       none
 *
 * @return      none
 */
void bspADCClose(void)
{
  // Release ADC resource
  Semaphore_post(Semaphore_handle(&mutex));
}


/*******************************************************************************
 * @fn          bspADCInit
 *
 * @brief       Initialize the RTOS ADC driver (must be called only once)
 *
 * @param       none
 *
 * @return      none
 */
void bspADCInit(void)
{

  Semaphore_Params semParams;

  // Create protection semaphore
  Semaphore_Params_init(&semParams);
  semParams.mode = Semaphore_Mode_BINARY;
  Semaphore_construct(&mutex, 1, &semParams);

  ADC_init();
  ADC_Params_init(&adcParams);
  
  adcHandle = ADC_open(Board_ADC, &adcParams);
  if (adcHandle == NULL)
    Task_exit();
}

/*******************************************************************************
 * @fn          bspADCRead
 *
 * @brief       Read the voltage in defined pin
 *
 * @param       none
 *
 * @return      Micro voltage
 */
uint32_t bspADCRead(void)
{
  static uint16_t raw;
  if(ADC_convert(adcHandle, &raw) == ADC_STATUS_SUCCESS)
    return ADC_convertRawToMicroVolts(adcHandle, raw);
  else
    return 0u;
}
/*********************************************************************
 * SYSTEM HOOK FUNCTIONS
 */


#endif

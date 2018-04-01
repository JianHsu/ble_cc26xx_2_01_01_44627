#ifndef AHRS_H
#define AHRS_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "Controller.h"
   
/*********************************************************************
 * CONSTANTS
 */
#define Status_Running 0x00
#define Status_Steady  0x01
#define Status_Sleep   0x02
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */
  
/*
 * Create the AHRS task
 */
void AHRS_createTask(void);

/*
 * Task Event Processor for characteristic changes
 */
void AHRS_processCharChangeEvt(uint8_t paramID);

/*
 * Task Event Processor for the BLE Application
 */
void AHRS_reset( void);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* AHRS_H */

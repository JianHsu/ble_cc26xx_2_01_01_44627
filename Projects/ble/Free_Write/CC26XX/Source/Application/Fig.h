#ifndef FIG_H
#define FIG_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "Controller.h"
#include "st_util.h"
   
/*********************************************************************
 * CONSTANTS
 */
#define FigCalCom       0x40
#define FigCount        5
#define FigKeyMask(x)   (0xffff^((x&0xff) | ((x&0xff)<<8)))
#define FigKey1         0x01

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */
  
/*
 * Create the Fig ang update \ask
 */
void ControllerFIG_init(void);

/*
 * Task Event Processor for characteristic changes
 */
void FIG_processCharChangeEvt(uint8_t paramID);

/*
 * Task Event Processor for the BLE Application
 */
void FIG_reset(void);

/*
 * Task Event Processor for Sensor reading
 */
void ControllerFig_processSensorEvent(void);

/*
 * Update reference angle
 */
void FIG_UpdRef(float Roll, float Ax, float Ay, float Az, float Mx, float My, float Mz);

/*
 * Update Keys Status
 */
void FIG_UpdKey();
/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* FIG_H */

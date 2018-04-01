#ifndef CONTROLLER_H
#define CONTROLLER_H


#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "ICall.h"
#include "peripheral.h"
#include <ti/sysbios/knl/Clock.h>
#include <ti/drivers/PIN.h>

/*********************************************************************
 * CONSTANTS
 */
// Service ID's for internal application use
#define SERVICE_ID_CC        0x09
#define SERVICE_ID_INERTIA   0x0C
#define SERVICE_ID_FIG       0x0D
#define SERVICE_ID_CAL       0x0E
 /*********************************************************************
 * MACROS
 */

#ifdef FEATURE_LCD
#define LCD_WRITE_STRING(s,x,y) devpkLcdText(s,x,y)
#define LCD_WRITES_STATUS(s) SensorTag_displayStatus(s)
#else
#define LCD_WRITE_STRING(s,x,y)
#define LCD_WRITES_STATUS(s)    System_printf("LCD:%s\n\r",s)
#endif
 
/*********************************************************************
 * VARIABLES
 */
extern ICall_Semaphore sem;
extern gaprole_States_t gapProfileState;

extern PIN_State pinGpioState;
extern PIN_Handle hGpioPin;

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task creation function for Controller
 */
extern void Controller_createTask(void);

/*
 * Function to call when a characteristic value has changed
 */
extern void Controller_charValueChangeCB(uint8_t sensorID, uint8_t paramID);

/*
 * Function to check the program stack
 */
extern void Controller_checkStack(void);

/*
 * Function to blink LEDs 'n' times
 */
extern void Controller_blinkLed(uint8_t led, uint8_t nBlinks);

/*
 * Update the advertising data with the latest key press status
 */
void Controller_updateAdvertisingData(uint8_t keyStatus);

void Controller_Capacitive_Touch_Config(uint8_t Enable);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
extern "C"
}
#endif

#endif  /* CONTROLLER_H */
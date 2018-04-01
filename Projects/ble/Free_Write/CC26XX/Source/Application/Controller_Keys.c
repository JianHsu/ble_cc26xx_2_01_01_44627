/*******************************************************************************
  Filename:       Controller_Keys.c
  Revised:        $Date: 2013-08-23 20:45:31 +0200 (fr, 23 aug 2013) $
  Revision:       $Revision: 35100 $

  Description:    This file contains the Sensor Tag sample application,
                  Keys part, for use with the TI Bluetooth Low 
                  Energy Protocol Stack.

  Copyright 2015  Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
*******************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "gatt.h"
#include "gattservapp.h"
#include "Controller_Keys.h"
#include "ioservice.h"

#include "Board.h"

#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
#define SK_KEY_REED             0x04
#define SK_PUSH_KEYS            0x03

// Key press time-outs (milliseconds)
#define POWER_PRESS_PERIOD      3000
#define RESET_PRESS_PERIOD      6000

/*********************************************************************
 * TYPEDEFS
 */
typedef struct
{
  uint32_t tStart;
  uint32_t tStop;
} KeyPress_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8_t keys;
static KeyPress_t key;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void processGapStateChange(void);

/*********************************************************************
 * PROFILE CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      ControllerKeys_init
 *
 * @brief   Initialization function for the Controller keys
 *
 */
void ControllerKeys_init(void)
{
  // Initialize the module state variables
  ControllerKeys_reset();
}

/*********************************************************************
 * @fn      ControllerKeys_processKeyLeft
 *
 * @brief   Interrupt handler for BUTTON 2 (left)
 *
 */
void ControllerKeys_processKey(void)
{
  if (PIN_getInputValue(Board_KEY))
  {
    keys &= ~SK_KEY;
    key.tStop = Clock_getTicks();
  }
  else
  {
    keys |= SK_KEY;
    key.tStart = Clock_getTicks();
  }
  
  // Wake up the application thread
  Semaphore_post(sem);
}

/*********************************************************************
 * @fn      ControllerKeys_processEvent
 *
 * @brief   Controller Keys event processor.  
 *
 */
void ControllerKeys_processEvent(void)
{
  static uint8_t current_keys = 0;
  if (current_keys != keys)
  {
    // Insert key state into advertising data
    if ( gapProfileState == GAPROLE_ADVERTISING )
    {
      Controller_updateAdvertisingData(keys);
    }
    
    // Check if key was pressed for more than 3 seconds and less than 6
    if ( (current_keys & SK_KEY)!=0 && (keys & SK_KEY)==0 )
    {
      if (gapProfileState == GAPROLE_CONNECTED)
      {
        int duration;
        
        duration = ((key.tStop - key.tStart) * Clock_tickPeriod) 
          / 1000;
        
        // Connected: change state after 3 second press (power/right button)
        if (duration > POWER_PRESS_PERIOD && duration < RESET_PRESS_PERIOD)
        {
          processGapStateChange();
        }
      }
      else
      {
        // Not connected; change state immediately (power/right button)
        processGapStateChange();
      } 
    }
  }
  current_keys = keys;
}

/*********************************************************************
 * @fn      ControllerKeys_reset
 *
 * @brief   Reset key state to 'not pressed'
 *
 * @param   none
 *
 * @return  none
 */
void ControllerKeys_reset(void)
{
  key.tStart = 0;
  key.tStop = 0;
  keys = 0;
}

/*********************************************************************
 * @fn      processGapStateChange
 *
 * @brief   Change the GAP state. 
 *          1. Connected -> disconnect and start advertising
 *          2. Advertising -> stop advertising
 *          3. Disconnected/not advertising -> start advertising
 *
 * @param   none
 *
 * @return  none
 */
static void processGapStateChange(void)
{
  if (gapProfileState != GAPROLE_CONNECTED)
  {
    uint8_t current_adv_enabled_status;
    uint8_t new_adv_enabled_status;
    
    // Find the current GAP advertising status
    GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status);
    
    if( current_adv_enabled_status == FALSE )
    {
      new_adv_enabled_status = TRUE;
    }
    else
    {
      new_adv_enabled_status = FALSE;
    }
    
    // Change the GAP advertisement status to opposite of current status
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof( uint8_t ), 
                         &new_adv_enabled_status);
  }
  
  if (gapProfileState == GAPROLE_CONNECTED)
  {
    uint8_t adv_enabled = TRUE;
    
    // Disconnect
    GAPRole_TerminateConnection();
    
    // Start advertising
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof( uint8_t ), 
                         &adv_enabled);
  }
}

/*********************************************************************
*********************************************************************/


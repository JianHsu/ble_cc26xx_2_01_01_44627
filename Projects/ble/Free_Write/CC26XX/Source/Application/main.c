// XDC, RTOS, driverlib
#include <xdc/runtime/Error.h>
#include <ti/sysbios/family/arm/cc26xx/Power.h>
#include <ti/sysbios/family/arm/cc26xx/PowerCC2650.h>
#include <ti/sysbios/BIOS.h>
#include <driverlib/vims.h>

// BLE
#include "Board.h"
#include "bcomdef.h"

#include "Controller.h"
#include "AHRS.h"

#ifndef USE_DEFAULT_USER_CFG

#include "./ICallBLE/bleUserConfig.h"

// BLE user defined configuration
bleUserCfg_t user0Cfg = BLE_USER_CFG;

#endif // USE_DEFAULT_USER_CFG
typedef struct { 
  unsigned int _r4;
  unsigned int _r5;
  unsigned int _r6;
  unsigned int _r7;
  unsigned int _r8;
  unsigned int _r9;
  unsigned int _r10;
  unsigned int _r11;
  unsigned int _r0;
  unsigned int _r1;
  unsigned int _r2;
  unsigned int _r3;
  unsigned int _r12;
  unsigned int _lr;
  unsigned int _pc;
  unsigned int _xpsr;
} execptionFrame;

#pragma optimize=none
void exceptionHandler(execptionFrame *e, unsigned int execLr)
{
  static __root unsigned int failPC = 0;
  static __root unsigned int lr = 0;
  failPC = e->_pc; // This is the Program Counter when the exception happened
  while(1);
}
int main()
{
  PIN_init(BoardGpioInitTable);
  #ifndef POWER_SAVING
  /* Set constraints for Standby and Idle mode */
  Power_setConstraint(Power_SB_DISALLOW);
  Power_setConstraint(Power_IDLE_PD_DISALLOW);
  #endif // POWER_SAVING
  /* Initialize ICall module */
  ICall_init();

  /* Start tasks of external images - Priority 5 */
  ICall_createRemoteTasks();

  /* Kick off profile - Priority 3 */
  GAPRole_createTask();

  /* Kick off application - Priority 1 */
  Controller_createTask();
  AHRS_createTask();
  
  BIOS_start();     /* enable interrupts and start SYS/BIOS */

  return 0;
}
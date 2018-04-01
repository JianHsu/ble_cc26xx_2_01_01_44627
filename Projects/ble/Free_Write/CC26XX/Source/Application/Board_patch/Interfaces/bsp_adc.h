#ifndef BSP_ADC_H
#define BSP_ADC_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "stdbool.h"
#include "stdint.h"

/*********************************************************************
 * CONSTANTS
 */
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * FUNCTIONS
 */
void bspADCInit(void);
uint32_t bspADCRead(void);
bool bspADCOpen(void);
void bspADCClose(void);

/////////////////////////////////////////////////////////////////////////////////
#ifdef __cplusplus
}
#endif

#endif /* BSP_ADC_H */

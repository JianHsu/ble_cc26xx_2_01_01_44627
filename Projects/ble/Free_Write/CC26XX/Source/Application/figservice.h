#ifndef FIGSERVICE_H
#define FIGSERVICE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "st_util.h"
  
/*********************************************************************
 * CONSTANTS
 */

// Service UUID
#define FIG_SERV_UUID         0xAA35
#define FIG_DATA_UUID         0xAA36
#define FIG_CONF_UUID         0xAA37
#define FIG_PERI_UUID         0xAA38
#define FIG_MAG_CAL1_UUID     0xAA39
#define FIG_MAG_CAL2_UUID     0xAA3A

// Length of sensor data in bytes
#define FIG_DATA_LEN          12
#define FIG_CAL_LEN1          15
#define FIG_CAL_LEN2          15

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * API FUNCTIONS
 */


/*
 * FIG_addService- Initializes the Sensor GATT Profile service by registering
 *          GATT attributes with the GATT server.
 */
extern bStatus_t FIG_addService(void);

/*
 * FIG_registerAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t FIG_registerAppCBs(sensorCBs_t *appCallbacks);

/*
 * FIG_setParameter - Set a Sensor GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    len   - length of data to write
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 */
extern bStatus_t FIG_setParameter(uint8_t param, uint8_t len, void *value);

/*
 * FIG_getParameter - Get a Sensor GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to read.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 */
extern bStatus_t FIG_getParameter(uint8_t param, void *value);


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* FIGSERVICE_H */
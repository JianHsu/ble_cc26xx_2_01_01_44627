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
#define Record_SERV_UUID         0xAA3C
#define Record_DATA_UUID         0xAA3D
#define Record_DATA2_UUID        0xAA3E

// Length of sensor data in bytes
#define Record_DATA_LEN          12

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
 * Record_addService- Initializes the Sensor GATT Profile service by registering
 *          GATT attributes with the GATT server.
 */
extern bStatus_t Record_addService(void);

/*
 * Record_registerAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t Record_registerAppCBs(sensorCBs_t *appCallbacks);

/*
 * Record_setParameter - Set a Sensor GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    len   - length of data to write
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 */
extern bStatus_t Record_setParameter(uint8_t param, uint8_t len, void *value);

/*
 * Record_getParameter - Get a Sensor GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to read.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 */
extern bStatus_t Record_getParameter(uint8_t param, void *value);


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* FIGSERVICE_H */
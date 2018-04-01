#ifndef AHRSSERVICE_H
#define AHRSSERVICE_H

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
#define AHRS_SERV_UUID         0xAA30
#define AHRS_DATA_UUID         0xAA31
#define AHRS_CONF_UUID         0xAA32
#define AHRS_PERI_UUID         0xAA33
#define AHRS_CALI_UUID         0xAA34
   
// Length of sensor data in bytes
// [][] [][] [][] [][][][] [][][][] [][][][] [][]
// Yaw Pitch Roll Dx       Dy       Dz       status1 status2
#define AHRS_DATA_LEN          20
   
#define AHRS_CALIB_LEN         12

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
 * AHRS_addService- Initializes the Sensor GATT Profile service by registering
 *          GATT attributes with the GATT server.
 */
extern bStatus_t AHRS_addService(void);

/*
 * AHRS_registerAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t AHRS_registerAppCBs(sensorCBs_t *appCallbacks);

/*
 * AHRS_setParameter - Set a Sensor GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    len   - length of data to write
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 */
extern bStatus_t AHRS_setParameter(uint8_t param, uint8_t len, void *value);

/*
 * AHRS_getParameter - Get a Sensor GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to read.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 */
extern bStatus_t AHRS_getParameter(uint8_t param, void *value);


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* AHRSSERVICE_H */
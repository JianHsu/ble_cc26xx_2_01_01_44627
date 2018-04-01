/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "linkdb.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "string.h"

#include "figservice.h"
#include "st_util.h"

#include <xdc/runtime/System.h>

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/* Service configuration values */
   
#define SENSOR_SERVICE_UUID     FIG_SERV_UUID
#define SENSOR_DATA_UUID        FIG_DATA_UUID
#define SENSOR_CONFIG_UUID      FIG_CONF_UUID
#define SENSOR_PERIOD_UUID      FIG_PERI_UUID
#define SENSOR_MAG_CAL1_UUID    FIG_MAG_CAL1_UUID
#define SENSOR_MAG_CAL2_UUID    FIG_MAG_CAL2_UUID
#define SENSOR_DATA_LEN         FIG_DATA_LEN
#define SENSOR_CAL_LEN1         FIG_CAL_LEN1
#define SENSOR_CAL_LEN2         FIG_CAL_LEN2


#define SENSOR_DATA_DESCR       "FIG. Data"
#define SENSOR_CONFIG_DESCR     "FIG. Conf."
#define SENSOR_PERIOD_DESCR     "FIG. Period."
#define SENSOR_MAG_CAL_DESCR    "Fig Mags Calbration in Int8."

#undef SENSOR_MIN_UPDATE_PERIOD
#define SENSOR_MIN_UPDATE_PERIOD  100 // Minimum 100 milliseconds

/*********************************************************************
 * TYPEDEFS
 */

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
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Service UUID
static CONST uint8_t sensorServiceUUID[TI_UUID_SIZE] =
{
  TI_UUID(SENSOR_SERVICE_UUID),
};

// Characteristic UUID: data
static CONST uint8_t sensorDataUUID[TI_UUID_SIZE] =
{
  TI_UUID(SENSOR_DATA_UUID),
};

// Characteristic UUID: config
static CONST uint8_t sensorCfgUUID[TI_UUID_SIZE] =
{
  TI_UUID(SENSOR_CONFIG_UUID),
};

// Characteristic UUID: period
static CONST uint8_t sensorPeriodUUID[TI_UUID_SIZE] =
{
  TI_UUID(SENSOR_PERIOD_UUID),
};

// Characteristic UUID: Mag meter Calibrations 1
static CONST uint8_t sensorCal1UUID[TI_UUID_SIZE] =
{
  TI_UUID(SENSOR_MAG_CAL1_UUID),
};

// Characteristic UUID: Mag meter Calibrations 2
static CONST uint8_t sensorCal2UUID[TI_UUID_SIZE] =
{
  TI_UUID(SENSOR_MAG_CAL2_UUID),
};


/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static sensorCBs_t *sensor_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// Profile Service attribute
static CONST gattAttrType_t sensorService = { TI_UUID_SIZE, sensorServiceUUID };

// Characteristic Value: data
static uint8_t sensorData[SENSOR_DATA_LEN] = {0};

// Characteristic Properties: data
static uint8_t sensorDataProps = GATT_PROP_READ | GATT_PROP_NOTIFY;


// Characteristic Configuration: data
static gattCharCfg_t *sensorDataConfig;

#ifdef USER_DESCRIPTION
// Characteristic User Description: data
static uint8_t sensorDataUserDescr[] = SENSOR_DATA_DESCR;
#endif

// Characteristic Properties: configuration
static uint8_t sensorCfgProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic Value: configuration
static uint8_t sensorCfg = 0;

#ifdef USER_DESCRIPTION
// Characteristic User Description: configuration
static uint8_t sensorCfgUserDescr[] = SENSOR_CONFIG_DESCR; 
#endif

// Characteristic Properties: period
static uint8_t sensorPeriodProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic Value: period
static uint8_t sensorPeriod = SENSOR_MIN_UPDATE_PERIOD / SENSOR_PERIOD_RESOLUTION;

#ifdef USER_DESCRIPTION
// Characteristic User Description: period
static uint8_t sensorPeriodUserDescr[] = SENSOR_PERIOD_DESCR;
#endif


// Characteristic Value: Calibration
static uint8_t MagCalbrations[SENSOR_CAL_LEN1] = {0};
static uint8_t MagCalbrations2[SENSOR_CAL_LEN2] = {0};

// Characteristic Properties: Calibration
static uint8_t sensorCalProps = GATT_PROP_READ | GATT_PROP_WRITE;
static uint8_t sensorCal2Props = GATT_PROP_READ | GATT_PROP_WRITE;


#ifdef USER_DESCRIPTION
// Characteristic User Description: configuration
static uint8_t sensorCalUserDescr[] = SENSOR_MAG_CAL_DESCR; 
#endif

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t sensorAttrTable[] =
{
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8_t *)&sensorService                 /* pValue */
  },

    // Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &sensorDataProps
    },

      // Characteristic Value "Data"
      {
        { TI_UUID_SIZE, sensorDataUUID },
        GATT_PERMIT_READ,
        0,
        sensorData
      },

      // Characteristic configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8_t *)&sensorDataConfig
      },

#ifdef USER_DESCRIPTION
      // Characteristic User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        sensorDataUserDescr
      },
#endif
    // Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &sensorCfgProps
    },

      // Characteristic Value "Configuration"
      {
        { TI_UUID_SIZE, sensorCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        &sensorCfg
      },

#ifdef USER_DESCRIPTION
      // Characteristic User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        sensorCfgUserDescr
      },
#endif
     // Characteristic Declaration "Period"
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &sensorPeriodProps
    },

      // Characteristic Value "Period"
      {
        { TI_UUID_SIZE, sensorPeriodUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        &sensorPeriod
      },

#ifdef USER_DESCRIPTION
      // Characteristic User Description "Period"
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        sensorPeriodUserDescr
      },
#endif
     // Characteristic Declaration "Calibration 1"
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &sensorCalProps
    },

      // Characteristic Value "Calibration 1"
      {
        { TI_UUID_SIZE, sensorCal1UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        MagCalbrations
      },

#ifdef USER_DESCRIPTION
      // Characteristic User Description "Calibration 1"
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        sensorCalUserDescr
      },
#endif
     // Characteristic Declaration "Calibration 2"
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &sensorCal2Props
    },

      // Characteristic Value "Calibration 2"
      {
        { TI_UUID_SIZE, sensorCal2UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        MagCalbrations2
      },

#ifdef USER_DESCRIPTION
      // Characteristic User Description "Calibration 2"
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        sensorCalUserDescr
      },
#endif
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t sensor_ReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                   uint8_t *pValue, uint16_t *pLen, 
                                   uint16_t offset, uint16_t maxLen,
                                   uint8_t method);
static bStatus_t sensor_WriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                    uint8_t *pValue, uint16_t len,
                                    uint16_t offset, uint8_t method);

/*********************************************************************
 * PROFILE CALLBACKS
 */
// FIG Profile Service Callbacks
static CONST gattServiceCBs_t sensorCBs =
{
  sensor_ReadAttrCB,  // Read callback function pointer
  sensor_WriteAttrCB, // Write callback function pointer
  NULL                // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      FIG_addService
 *
 * @brief   Initializes the FIG Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 */
bStatus_t FIG_addService(void)
{
  // Allocate Client Characteristic Configuration table
  sensorDataConfig = (gattCharCfg_t *)ICall_malloc(sizeof(gattCharCfg_t) *
                                                    linkDBNumConns);
  if (sensorDataConfig == NULL)
  {
    return (bleMemAllocError);
  }
  
  
  // Register with Link DB to receive link status change callback
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, sensorDataConfig);

  // Register GATT attribute list and CBs with GATT Server App
  return GATTServApp_RegisterService( sensorAttrTable,
                                      GATT_NUM_ATTRS (sensorAttrTable),
                                      GATT_MAX_ENCRYPT_KEY_SIZE,
                                      &sensorCBs );
}


/*********************************************************************
 * @fn      FIG_registerAppCBs
 *
 * @brief   Registers the application callback function. Only call
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t FIG_registerAppCBs(sensorCBs_t *appCallbacks)
{
  if (sensor_AppCBs == NULL)
  {
    if (appCallbacks != NULL)
    {
      sensor_AppCBs = appCallbacks;
    }

    return (SUCCESS);
  }

  return (bleAlreadyInRequestedMode);
}

/*********************************************************************
 * @fn      FIG_setParameter
 *
 * @brief   Set a parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to write
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 *
 * @return  bStatus_t
 */
bStatus_t FIG_setParameter(uint8_t param, uint8_t len, void *value)
{
  bStatus_t ret = SUCCESS;

  switch (param)
  {
    case SENSOR_DATA:
    if (len == SENSOR_DATA_LEN)
    {
      memcpy(sensorData, value, SENSOR_DATA_LEN);
      // See if Notification has been enabled
      ret = GATTServApp_ProcessCharCfg(sensorDataConfig, sensorData, FALSE,
                                 sensorAttrTable, 
                                 GATT_NUM_ATTRS(sensorAttrTable),
                                 INVALID_TASK_ID, sensor_ReadAttrCB);
    }
    else
    {
      ret = bleInvalidRange;
    }
    break;

    case SENSOR_CONF:
      if (len == sizeof(uint8_t))
      {
        sensorCfg = *((uint8_t*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case SENSOR_PERI:
      if (len == sizeof(uint8_t))
      {
        sensorPeriod = *((uint8_t*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case SENSOR_CAL_DATA1:
      if (len == SENSOR_CAL_LEN1)
      {
        memcpy(MagCalbrations, value, SENSOR_CAL_LEN1);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
      
    case SENSOR_CAL_DATA2:
      if (len == SENSOR_CAL_LEN2)
      {
        memcpy(MagCalbrations2, value, SENSOR_CAL_LEN2);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return (ret);
}

/*********************************************************************
 * @fn      FIG_getParameter
 *
 * @brief   Get a Sensor Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 *
 * @return  bStatus_t
 */
bStatus_t FIG_getParameter(uint8_t param, void *value)
{
  bStatus_t ret = SUCCESS;

  switch (param)
  {
    case SENSOR_DATA:
      memcpy(value, sensorData, SENSOR_DATA_LEN);
      break;

    case SENSOR_CONF:
      *((uint8_t*)value) = sensorCfg;
      break;

    case SENSOR_PERI:
      *((uint8_t*)value) = sensorPeriod;
      break;
      
    case SENSOR_CAL_DATA1:
      memcpy(value, MagCalbrations, SENSOR_CAL_LEN1);
      break;  
      
    case SENSOR_CAL_DATA2:
      memcpy(value, MagCalbrations2, SENSOR_CAL_LEN2);
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return (ret);
}

/*********************************************************************
 * @fn          sensor_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message 
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t sensor_ReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                   uint8_t *pValue, uint16_t *pLen, 
                                   uint16_t offset, uint16_t maxLen,
                                   uint8_t method)
{
  uint16_t uuid;
  bStatus_t status = SUCCESS;
  // If attribute permissions require authorization to read, return error
  if (gattPermitAuthorRead(pAttr->permissions))
  {
    // Insufficient authorization
    return (ATT_ERR_INSUFFICIENT_AUTHOR);
  }

  // Make sure it's not a blob operation (no attributes in the profile are long)
  if (offset > 0)
  {
    return (ATT_ERR_ATTR_NOT_LONG);
  }

  if (utilExtractUuid16(pAttr,&uuid) == FAILURE) {
    // Invalid handle
    *pLen = 0;
    return ATT_ERR_INVALID_HANDLE;
  }

  switch (uuid)
  {
    // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
    // gattserverapp handles those reads
    case SENSOR_DATA_UUID:
      *pLen = SENSOR_DATA_LEN;
      memcpy(pValue, pAttr->pValue, SENSOR_DATA_LEN);
      break;

    case SENSOR_CONFIG_UUID:
    case SENSOR_PERIOD_UUID:
      *pLen = 1;
      *pValue = *pAttr->pValue;
      break;
    
    case SENSOR_MAG_CAL1_UUID:
      *pLen = SENSOR_CAL_LEN1;
      memcpy(pValue, pAttr->pValue, SENSOR_CAL_LEN1);
      break;
      
    case SENSOR_MAG_CAL2_UUID:
      *pLen = SENSOR_CAL_LEN2;
      memcpy(pValue, pAttr->pValue, SENSOR_CAL_LEN2);
      break;
      
    default:
      *pLen = 0;
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
    }

  return (status);
}

/*********************************************************************
 * @fn      sensor_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message 
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t sensor_WriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                    uint8_t *pValue, uint16_t len,
                                    uint16_t offset, uint8_t method)
{
  bStatus_t status = SUCCESS;
  uint8_t notifyApp = 0xFF;
  uint16_t uuid;

  // If attribute permissions require authorization to write, return error
  if (gattPermitAuthorWrite(pAttr->permissions))
  {
    // Insufficient authorization
    return (ATT_ERR_INSUFFICIENT_AUTHOR);
  }

  if (utilExtractUuid16(pAttr,&uuid) == FAILURE) {
    // Invalid handle
    return ATT_ERR_INVALID_HANDLE;
  }

  switch (uuid)
  {
    case SENSOR_DATA_UUID:
      // Should not get here
      break;

    case SENSOR_CONFIG_UUID:
      // Validate the value
      // Make sure it's not a blob oper
      if (offset == 0)
      {
        if (len != 1) 
        {
          status = ATT_ERR_INVALID_VALUE_SIZE;
        }
      }
      else
      {
        status = ATT_ERR_ATTR_NOT_LONG;
      }

      // Write the value
      if (status == SUCCESS)
      {
        uint8_t *pCurValue = (uint8_t *)pAttr->pValue;

        *pCurValue = pValue[0];

        if (pAttr->pValue == &sensorCfg)
        {
          notifyApp = SENSOR_CONF;
        }
      }
      break;

    case SENSOR_PERIOD_UUID:
      // Validate the value
      // Make sure it's not a blob oper
      if (offset == 0)
      {
        if (len != 1) 
        {
          status = ATT_ERR_INVALID_VALUE_SIZE;
        }
      }
      else
      {
        status = ATT_ERR_ATTR_NOT_LONG;
      }
      // Write the value
      if (status == SUCCESS)
      {
        if (pValue[0]>=(10/SENSOR_PERIOD_RESOLUTION))
        {

          uint8_t *pCurValue = (uint8_t *)pAttr->pValue;
          *pCurValue = pValue[0];

          if (pAttr->pValue == &sensorPeriod)
          {
            notifyApp = SENSOR_PERI;
          }
        }
        else
        {
           status = ATT_ERR_INVALID_VALUE;
        }
      }
      break;

    case SENSOR_MAG_CAL1_UUID:
      if (offset == 0)
      {
        if (len != SENSOR_CAL_LEN1) 
        {
          status = ATT_ERR_INVALID_VALUE_SIZE;
        }
      }
      else
      {
        status = ATT_ERR_ATTR_NOT_LONG;
      }
      // Write the value
      if (status == SUCCESS)
      {
        memcpy(pAttr->pValue, pValue, SENSOR_CAL_LEN1);
        notifyApp = SENSOR_CAL_DATA1;
      }
      break;
      
    case SENSOR_MAG_CAL2_UUID:
      if (offset == 0)
      {
        if (len != SENSOR_CAL_LEN2)
        {
          status = ATT_ERR_INVALID_VALUE_SIZE;
        }
      }
      else
      {
        status = ATT_ERR_ATTR_NOT_LONG;
      }
      // Write the value
      if (status == SUCCESS)
      {
        memcpy(pAttr->pValue, pValue, SENSOR_CAL_LEN2);
        notifyApp = SENSOR_CAL_DATA2;
      }
      break;
      
    case GATT_CLIENT_CHAR_CFG_UUID:
      status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len,
                                              offset, GATT_CLIENT_CFG_NOTIFY);
      break;

    default:
      // Should never get here!
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
  }

  // If a characteristic value changed then callback function 
  // to notify application of change
  if ((notifyApp != 0xFF ) && sensor_AppCBs && sensor_AppCBs->pfnSensorChange)
  {
    sensor_AppCBs->pfnSensorChange(notifyApp);
  }

  return (status);
}

/*********************************************************************
*********************************************************************/

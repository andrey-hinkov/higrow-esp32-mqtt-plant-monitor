#ifndef DRVSENS_H
#define DRVSENS_H

/*******************/
/* Includes        */
/*******************/

#include <Arduino.h>

/******************/
/* External types */
/******************/

typedef enum{ /**< Enum type for all installed sensors; add/remove sensors as needed */
  DRVSENS_TEMPERATURE_SENSOR_ID = 0u, /**< DHT11 temperature */
  DRVSENS_HUMIDITY_SENSOR_ID,         /**< DHT11 humidity */
  DRVSENS_SOIL_MOISTURE_SENSOR_ID,    /**< Soil moisture sensor */
  DRVSENS_SENSOR_COUNT                /**< Number of all installed sensors */
} DrvSens_ESensorID;

/************************************/
/* Prototypes of external functions */
/************************************/

bool DrvSens_initAllSensors(void); /**< Function to initialize all installed sensors */
bool DrvSens_readAllSensors(void); /**< Function to read all installed sensors */
String DrvSens_getSensorReadingAsString(uint8_t u8SensorID); /**< Function to get sensor reading (value + unit) as string */

#endif /* DRVSENS_H */

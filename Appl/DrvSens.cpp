/****************************************/
/* Driver component for reading sensors */
/* Version: 0.1                         */
/****************************************/

/*******************/
/* Includes        */
/*******************/

#include "DrvSens.h" /**< Own header file */
#include <DHT.h>     /**< Library for reading DHT11/DHT22 sensors */

/*******************/
/* Config          */
/*******************/

// TODO: it would be better to have some of the sensor calibration data
// in non-volatile memory (flashable parameters) rather than macros

// TODO: support temperature in Fahrenheit

/* DHT sensor config */

#define DRVSENS_DHT_TYPE  DHT11 /**< Specify DHT sensor type: DHT11 or DHT22 */

#define DRVSENS_DHT_DATA_PIN  (22u)  /**< Specify the data pin number of the DHT sensor */

#if (DRVSENS_DHT_TYPE == DHT11)
  #define DRVSENS_MIN_TEMPERATURE_LIMIT_CELSIUS (0.0f)   /**< Min temperature limit of DHT11 sensor */
  #define DRVSENS_MAX_TEMPERATURE_LIMIT_CELSIUS (50.0f)  /**< Max temperature limit of DHT11 sensor */
  #define DRVSENS_MIN_HUMIDITY_LIMIT_PERCENT    (20.0f)  /**< Min humidity limit of DHT11 sensor */
  #define DRVSENS_MAX_HUMIDITY_LIMIT_PERCENT    (90.0f)  /**< Max humidity limit of DHT11 sensor */
#elif (DRVSENS_DHT_TYPE == DHT22)
  #define DRVSENS_MIN_TEMPERATURE_LIMIT_CELSIUS (-40.0f) /**< Min temperature limit of DHT22 sensor */
  #define DRVSENS_MAX_TEMPERATURE_LIMIT_CELSIUS (80.0f)  /**< Max temperature limit of DHT22 sensor */
  #define DRVSENS_MIN_HUMIDITY_LIMIT_PERCENT    (0.0f)   /**< Min humidity limit of DHT22 sensor */
  #define DRVSENS_MAX_HUMIDITY_LIMIT_PERCENT    (100.0f) /**< Max humidity limit of DHT22 sensor */
#else
  #error "DRVSENS_DHT_TYPE unknown"
#endif /*DRVSENS_DHT_TYPE*/

#define DRVSENS_TEMPERATURE_SENSOR_NAME_INTERNAL  "TEMPERATURE_SENSOR" /**< Name of temperature sensor for internal use */
#define DRVSENS_TEMPERATURE_SENSOR_UNIT           "C"                  /**< Unit designator of temperature sensor */

#define DRVSENS_HUMIDITY_SENSOR_NAME_INTERNAL     "HUMIDITY_SENSOR"    /**< Name of humidity sensor for internal use */
#define DRVSENS_HUMIDITY_SENSOR_UNIT              "%"                  /**< Unit designator of humidity sensor */

/* Soil moisture sensor config */

#define DRVSENS_SOIL_MOISTURE_SENSOR_DATA_PIN (32u)  /**< Specify the data pin of the soil moisture sensor */

#define DRVSENS_MIN_SOIL_MOISTURE_RAW (3275u)        /**< Min soil moisture limit of the soil moisture sensor (completely dry) */
#define DRVSENS_MAX_SOIL_MOISTURE_RAW (1482u)        /**< Max soil moisture limit of the soil moisture sensor (completely wet) */

#define DRVSENS_LOWER_BOUND_SOIL_MOISTURE  (0u)      /**< Used to convert raw soil moisture reading to percentage */
#define DRVSENS_UPPER_BOUND_SOIL_MOISTURE  (1000u)   /**< Used to convert raw soil moisture reading to percentage */
#define DRVSENS_SOIL_MOISTURE_SCALE_FACTOR (10.0f)   /**< Used to convert raw soil moisture reading to percentage */

#define DRVSENS_SOIL_MOISTURE_SENSOR_NAME_INTERNAL  "SOIL_MOISTURE_SENSOR" /**< Name of soil moisture sensor for internal use */
#define DRVSENS_SOIL_MOISTURE_SENSOR_UNIT           "%"                    /**< Unit designator of soil moisture sensor */

/* Common config */

#define DRVSENS_MAX_READ_ATTEMPTS   (3u)    /**< Specify max number of attempts to read a sensor */
#define DRVSENS_READ_RETRY_DELAY_MS (3000u) /**< Specify delay in ms between retries if a sensor read attempt has failed */
#define DRVSENS_1000MS (1000u)

/******************/
/* Internal types */
/******************/

typedef struct{ /**< Return type of sensor readers */
  float fValue;
  bool  isValid;
} DrvSens_SResult;

typedef bool (* const DrvSens_fpSensorInitializer)(void); /**< Function pointer type of sensor initializers */
typedef const DrvSens_SResult* const (* const DrvSens_fpSensorReader)(void); /**< Function pointer type of sensor readers */

/************************************/
/* Prototypes of internal functions */
/************************************/

/* Sensor initializers */
static bool DrvSens_initTemperatureSensor(void);
static bool DrvSens_initHumiditySensor(void);
static bool DrvSens_initSoilMoistureSensor(void);

/* Sensor readers */ 
static const DrvSens_SResult* const DrvSens_readTemperatureSensor(void);
static const DrvSens_SResult* const DrvSens_readHumiditySensor(void);
static const DrvSens_SResult* const DrvSens_readSoilMoistureSensor(void);

static bool DrvSens_attemptToReadSensor(uint8_t u8SensorID);

/***********/
/* Globals */
/***********/

/* Function pointers tables */

/* Array of function pointers to sensor initializers, order this in sync with DrvSens_ESensorID */
static DrvSens_fpSensorInitializer DrvSens_afpSensorInitializers[DRVSENS_SENSOR_COUNT] = {&DrvSens_initTemperatureSensor,
                                                                                          &DrvSens_initHumiditySensor,
                                                                                          &DrvSens_initSoilMoistureSensor};

/* Array of function pointers to sensor readers, order this in sync with DrvSens_ESensorID */
static DrvSens_fpSensorReader DrvSens_afpSensorReaders[DRVSENS_SENSOR_COUNT] = {&DrvSens_readTemperatureSensor,
                                                                                &DrvSens_readHumiditySensor,
                                                                                &DrvSens_readSoilMoistureSensor};

/* Data storage */ 

static float DrvSens_afReadoutValues[DRVSENS_SENSOR_COUNT]; /**< Array to store readout values for all sensors */

/* Sensor objects */

DHT dhtSensor(DRVSENS_DHT_DATA_PIN, DRVSENS_DHT_TYPE);  /**< DHT sensor object */

/* Sensor attributes */

/* Array with sensor names for internal use */
static const String DrvSens_asSensorNames[DRVSENS_SENSOR_COUNT] = {DRVSENS_TEMPERATURE_SENSOR_NAME_INTERNAL,
                                                                   DRVSENS_HUMIDITY_SENSOR_NAME_INTERNAL,
                                                                   DRVSENS_SOIL_MOISTURE_SENSOR_NAME_INTERNAL};
/* Array with sensor unit designators */
static const String DrvSens_asSensorUnits[DRVSENS_SENSOR_COUNT] = {DRVSENS_TEMPERATURE_SENSOR_UNIT,
                                                                   DRVSENS_HUMIDITY_SENSOR_UNIT,
                                                                   DRVSENS_SOIL_MOISTURE_SENSOR_UNIT};

/*************************************/
/* Definitions of internal functions */
/*************************************/

/* Function to initialize temperature sensor    */
/* param[in]:     none                          */
/* param[return]: isSuccess                     */
static bool DrvSens_initTemperatureSensor(void){
  dhtSensor.begin();
  return true;
}

/* Function to initialize humidity sensor       */
/* param[in]:     none                          */
/* param[return]: isSuccess                     */
static bool DrvSens_initHumiditySensor(void){
  return true;
}

/* Function to initialize soil moisture sensor  */
/* param[in]:     none                          */
/* param[return]: isSuccess                     */
static bool DrvSens_initSoilMoistureSensor(void){
  return true;
}

/* Function to read temperature sensor          */
/* param[in]:     none                          */
/* param[return]: pointer to result             */
static const DrvSens_SResult* const DrvSens_readTemperatureSensor(void){
  static DrvSens_SResult Result;

  // Read temperature sensor
  Result.fValue = dhtSensor.readTemperature();

  // Check if temperature reading is valid
  if((isnan(Result.fValue)) ||
     (Result.fValue < DRVSENS_MIN_TEMPERATURE_LIMIT_CELSIUS) ||
     (Result.fValue > DRVSENS_MAX_TEMPERATURE_LIMIT_CELSIUS)){
      
        Result.isValid = false;
        
  } else {
    Result.isValid = true;
  }

  return &Result;  
}

/* Function to read humidity sensor             */
/* param[in]:     none                          */
/* param[return]: pointer to result             */
static const DrvSens_SResult* const DrvSens_readHumiditySensor(void){
  static DrvSens_SResult Result;

  // Read humidity sensor
  Result.fValue = dhtSensor.readHumidity();

  // Check if humidity reading is valid
  if((isnan(Result.fValue)) ||
     (Result.fValue < DRVSENS_MIN_HUMIDITY_LIMIT_PERCENT) ||
     (Result.fValue > DRVSENS_MAX_HUMIDITY_LIMIT_PERCENT)){
      
        Result.isValid = false;
        
  } else {
    Result.isValid = true;
  }

  return &Result;  
}

/* Function to read soil moisture sensor        */
/* param[in]:     none                          */
/* param[return]: pointer to result             */
static const DrvSens_SResult* const DrvSens_readSoilMoistureSensor(void){
  static DrvSens_SResult Result;

  // TODO: this is not very accurate, come up with a better algorithm
  // to evaluate the raw reading of this sensor
  
  // Read soil moisture sensor
  uint16_t u16RawValue = analogRead(DRVSENS_SOIL_MOISTURE_SENSOR_DATA_PIN);

  // Check if soil moisture reading is valid
  if((u16RawValue > DRVSENS_MIN_SOIL_MOISTURE_RAW) ||
     (u16RawValue < DRVSENS_MAX_SOIL_MOISTURE_RAW)){
      
        Result.isValid = false;
        
  } else {
    
    Result.isValid = true;

    // Convert raw value to percentage
    Result.fValue = map(u16RawValue,
                      DRVSENS_MIN_SOIL_MOISTURE_RAW,
                      DRVSENS_MAX_SOIL_MOISTURE_RAW,
                      DRVSENS_LOWER_BOUND_SOIL_MOISTURE,
                      DRVSENS_UPPER_BOUND_SOIL_MOISTURE)/DRVSENS_SOIL_MOISTURE_SCALE_FACTOR;
  }

  return &Result;  
}

/* Function to attempt to read a sensor  */
/* param[in]:     u8SensorID             */
/* param[return]: isSuccess              */
static bool DrvSens_attemptToReadSensor(uint8_t u8SensorID){

    uint8_t u8ReadAttemptCount = 0u;
    const DrvSens_SResult* pResult;

    do{

      Serial.printf("Reading SensorID %d (%s): ", u8SensorID, DrvSens_asSensorNames[u8SensorID].c_str());
  
      // Exit if the reader function of the sensor doesn't exist
      if(DrvSens_afpSensorReaders[u8SensorID] == NULL){
        Serial.printf("FAIL\n");
        return false;
      }
            
      // Call the sensor reader and get the pointer to the result
      pResult = DrvSens_afpSensorReaders[u8SensorID]();
  
      // Check if the result is valid
      if(pResult != NULL){
        if(pResult->isValid){
          
          // Transfer the readout value to the storage aray and exit
          DrvSens_afReadoutValues[u8SensorID] = pResult->fValue;

          Serial.printf("%s\n", DrvSens_getSensorReadingAsString(u8SensorID).c_str());
          
          return true;
        }
      }

      // If read attempt failed, try again
      if(u8ReadAttemptCount == DRVSENS_MAX_READ_ATTEMPTS-1u){
        Serial.printf("FAIL, abort\n");
        return false;
      } else {
        Serial.printf("FAIL, retry in %ds...\n", DRVSENS_READ_RETRY_DELAY_MS/DRVSENS_1000MS);
      }
      delay(DRVSENS_READ_RETRY_DELAY_MS);
      u8ReadAttemptCount++;
      
    } while (u8ReadAttemptCount < DRVSENS_MAX_READ_ATTEMPTS);
}

/*************************************/
/* Definitions of external functions */
/*************************************/

/* Function to initialize all installed sensors */
/* param[in]:     none                          */
/* param[return]: isSuccess                     */
bool DrvSens_initAllSensors(void){

  // Sweep through all installed sensors and call their initializers
  for(uint8_t u8SensorID = 0; u8SensorID < (uint8_t)DRVSENS_SENSOR_COUNT; u8SensorID++){
    
    Serial.printf("Initializing SensorID %d (%s): ", u8SensorID, DrvSens_asSensorNames[u8SensorID].c_str());
    
    // Exit if the initializer function of the sensor doesn't exist
    if(DrvSens_afpSensorInitializers[u8SensorID] == NULL){
      Serial.printf("FAIL\n");
      return false;
    }

    // Exit if the initializer function of the sensor fails
    if(!DrvSens_afpSensorInitializers[u8SensorID]()){
      Serial.printf("FAIL\n");
      return false;
    }

    Serial.printf("OK\n");
   
  }

  return true;  
}

/* Function to read all installed sensors     */
/* param[in]:     none                        */
/* param[return]: isSuccess                   */
bool DrvSens_readAllSensors(void){
  
  // Sweep through all installed sensors and attempt to read them
  for(uint8_t u8SensorID = 0; u8SensorID < (uint8_t)DRVSENS_SENSOR_COUNT; u8SensorID++){
    
    // Exit if a sensor could not be read
    if(!DrvSens_attemptToReadSensor(u8SensorID)){
      return false;
    };
    
  }

  return true; 
}

/* Function to get sensor reading (value + unit) as string */
/* param[in]:     u8SensorID                               */
/* param[return]: string                                   */
String DrvSens_getSensorReadingAsString(uint8_t u8SensorID){
  return (String(DrvSens_afReadoutValues[u8SensorID]) + String(DrvSens_asSensorUnits[u8SensorID]));
}

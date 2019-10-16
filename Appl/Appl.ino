/****************************************************/
/* Appl component: read sensors and publish to MQTT */
/* Version: 0.1                                     */
/****************************************************/

/*******************/
/* Includes        */
/*******************/

#include "DrvSens.h"  /**< Driver component for reading sensors */
#include "DrvWiFi.h"  /**< Driver component for WiFi */
#include "SvcMQTT.h"  /**< Service component for MQTT */
#include "SvcSleep.h" /**< Service component for deep sleep */

/*******************/
/* Config          */
/*******************/

/* LED */
#define APPL_LED_PIN               (16u)  /**< LED pin */
#define APPL_LED_FLASH_DURATION_MS (100u) /**< LED flash duration in ms */
#define APPL_LED_FLASH_TIMES       (2u)   /**< Times to flash the LED */

/* Sleep */
#define APPL_TIME_TO_SLEEP_NORMAL_S (300u) /**< Time to sleep between sensor readings in normal operation (seconds) */
#define APPL_TIME_TO_SLEEP_ERROR_S  (900u) /**< Time to sleep between sensor readings when an error had previously occured (seconds)*/

/* Serial */
#define APPL_SERIAL_BAUDRATE (115200u) /**< Serial baudrate (used for debug messages) */

/* WiFi */
#define APPL_WIFI_SSID     "your_wifi_ssid" /**< WiFi network SSID */
#define APPL_WIFI_PASSWORD "your_wifi_pass" /**< WiFi network password */

/* MQTT */
#define APPL_MQTT_SERVER   "broker.hivemq.com" /**< MQTT server name */
#define APPL_MQTT_PORT     (1883u)             /**< MQTT port */
#define APPL_MQTT_ID       "esp32Device"       /**< MQTT id */
#define APPL_MQTT_USER     ""                  /**< MQTT username */
#define APPL_MQTT_PASSWORD ""                  /**< MQTT password */

#define APPL_MQTT_TOPIC_TEMPERATURE    "esp32Device/temperature"    /**< MQTT topic for temperature sensor */
#define APPL_MQTT_TOPIC_HUMIDITY       "esp32Device/humidity"       /**< MQTT topic for humidity sensor */
#define APPL_MQTT_TOPIC_SOIL_MOISTURE  "esp32Device/soil_moisture"  /**< MQTT topic for soil moisture sensor */

#define APPL_MQTT_TOPIC_ERROR_MESSAGES "esp32Device/error_messages" /**< MQTT topic for error messages */

/* Front-face names for sensor readings */
#define APPL_TEMPERATURE_SENSOR_NAME_PRETTY    "Temperature: "      /**< Front-face name for temperature sensor */
#define APPL_HUMIDITY_SENSOR_NAME_PRETTY       "Humidity: "         /**< Front-face name for humidity sensor */
#define APPL_SOIL_MOISTURE_SENSOR_NAME_PRETTY  "Soil moisture: "    /**< Front-face name for soil moisture sensor */

/***********/
/* Globals */
/***********/

/* Array with front-face names for sensor readings */
static const String Appl_asSensorNamesPretty[DRVSENS_SENSOR_COUNT] = {APPL_TEMPERATURE_SENSOR_NAME_PRETTY,
                                                                      APPL_HUMIDITY_SENSOR_NAME_PRETTY,
                                                                      APPL_SOIL_MOISTURE_SENSOR_NAME_PRETTY};

/* Array with MQTT topcis for sensors */
static const String Appl_asMQTTSensorTopics[DRVSENS_SENSOR_COUNT] = {APPL_MQTT_TOPIC_TEMPERATURE,
                                                                     APPL_MQTT_TOPIC_HUMIDITY,
                                                                     APPL_MQTT_TOPIC_SOIL_MOISTURE};
                                                                           
/************************************/
/* Prototypes of internal functions */
/************************************/

static void Appl_flashLed(uint8_t u8LedPin, uint16_t u16DurationMs, uint8_t u8TimesToFlash);
static bool Appl_publishSensorsDataToMQTT();

/*************************************/
/* Definitions of internal functions */
/*************************************/

/* Function to flash a LED                             */
/* param[in]:     LedPin, Duration in ms, TimesToFlash */
/* param[return]: none                                 */
static void Appl_flashLed(uint8_t u8LedPin, uint16_t u16DurationMs, uint8_t u8TimesToFlash){

    pinMode(u8LedPin, OUTPUT);

    for(uint8_t i = 0u; i<u8TimesToFlash; i++){
      digitalWrite(u8LedPin, LOW);
      delay(u16DurationMs);
      digitalWrite(u8LedPin, HIGH);
      delay(u16DurationMs);
    }
}

/* Function to publish sensors data to MQTT */
/* param[in]:     none                      */
/* param[return]: isSuccess                 */
static bool Appl_publishSensorsDataToMQTT(){

  for(uint8_t u8SensorID = 0; u8SensorID < (uint8_t)DRVSENS_SENSOR_COUNT; u8SensorID++){
  
    if(!SvcMQTT_publish(Appl_asMQTTSensorTopics[u8SensorID].c_str(), // topic
                        String(Appl_asSensorNamesPretty[u8SensorID] + DrvSens_getSensorReadingAsString(u8SensorID)).c_str(), // payload           
                        false)){ //isRetained
      return false;
    }        
  }

   return true;
}

/*******************/
/* Setup           */
/*******************/

void setup(){

  // TODO: add watchdog
  
  // Flash the LED when the board turns on
  Appl_flashLed(APPL_LED_PIN, APPL_LED_FLASH_DURATION_MS, APPL_LED_FLASH_TIMES);
  
  // Init serial
  // TODO: implement switching debug messages on/off
  Serial.begin(APPL_SERIAL_BAUDRATE);

  // Init WiFi
  if(!DrvWiFi_init(APPL_WIFI_SSID, APPL_WIFI_PASSWORD)){
    // On fail go to sleep for extended time
    SvcSleep_goToSleep(APPL_TIME_TO_SLEEP_ERROR_S);    
  }

  // Init MQTT
  if(!SvcMQTT_init(APPL_MQTT_SERVER,
                   APPL_MQTT_PORT,
                   APPL_MQTT_ID,
                   APPL_MQTT_USER,
                   APPL_MQTT_PASSWORD)){
    // On fail go to sleep for extended time
    SvcSleep_goToSleep(APPL_TIME_TO_SLEEP_ERROR_S);      
  }

  // Init all sensors
  if(!DrvSens_initAllSensors()){
    // Publish on MQTT error topic
    (void)SvcMQTT_publish(APPL_MQTT_TOPIC_ERROR_MESSAGES, "Sensor fault", false);
    // On fail go to sleep for extended time
    SvcSleep_goToSleep(APPL_TIME_TO_SLEEP_ERROR_S);
  };
  
  // Read all sensors
  if(!DrvSens_readAllSensors()){
    // Publish on MQTT error topic
    (void)SvcMQTT_publish(APPL_MQTT_TOPIC_ERROR_MESSAGES, "Sensor fault", false);
    // On fail go to sleep for extended time
    SvcSleep_goToSleep(APPL_TIME_TO_SLEEP_ERROR_S);
  };

  // Publish sensors data to MQTT
  if(!Appl_publishSensorsDataToMQTT()){
    // On fail go to sleep for extended time
    SvcSleep_goToSleep(APPL_TIME_TO_SLEEP_ERROR_S);
  }

  // All good, go to sleep for normal time
  SvcSleep_goToSleep(APPL_TIME_TO_SLEEP_NORMAL_S);
}

/*******************/
/* Loop            */
/*******************/

void loop(){
  /* do nothing */
}

/****************************************/
/* Driver component for WiFi            */
/* Version: 0.1                         */
/****************************************/

/*******************/
/* Includes        */
/*******************/

#include "DrvWiFi.h" /**< Own header file */
#include <WiFi.h>    /**< WiFi library */

/*******************/
/* Config          */
/*******************/

#define DRVWIFI_CONNECT_TIMEOUT_MS     (5000u) /**< Specify max time to wait for a succesfull WiFi connection */
#define DRVWIFI_MAX_CONNECT_ATTEMPTS   (3u)    /**< Specify max number of attempts to connect to WiFi network */
#define DRVWIFI_CONNECT_RETRY_DELAY_MS (5000u) /**< Specify delay in ms between retries if a connect attempt has failed */
#define DRVWIFI_100MS  (100u)
#define DRVWIFI_1000MS (1000u)

/*************************************/
/* Definitions of external functions */
/*************************************/

/* Function to init WiFi connection     */
/* param[in]:     SSID, Password        */
/* param[return]: isSuccess             */
bool DrvWiFi_init(const char* sSSID, const char* sPassword){

  uint8_t u8ConnectAttemptCount = 0u;
  uint16_t u16ElapsedTime = 0u;

  do{

    Serial.printf("Connecting to WiFi network \"%s\" : ", sSSID);

    WiFi.begin(sSSID, sPassword);

    while(u16ElapsedTime < DRVWIFI_CONNECT_TIMEOUT_MS){
    
      // Check for a succesfull connection every 100ms
      delay(DRVWIFI_100MS);
      u16ElapsedTime += DRVWIFI_100MS;

      if(WiFi.status()==WL_CONNECTED){
        Serial.print("OK (IP:");
        Serial.print(WiFi.localIP());
        Serial.println(")");
        return true;
      } 
    }

    // If connect attempt failed, try again
    if(u8ConnectAttemptCount == DRVWIFI_MAX_CONNECT_ATTEMPTS-1u){
      Serial.printf("FAIL, abort\n");
      return false;
    } else {
      Serial.printf("FAIL, retry in %ds...\n", DRVWIFI_CONNECT_RETRY_DELAY_MS/DRVWIFI_1000MS);
    }
    delay(DRVWIFI_CONNECT_RETRY_DELAY_MS);
    u8ConnectAttemptCount++;
    u16ElapsedTime = 0u;
    
  } while (u8ConnectAttemptCount < DRVWIFI_MAX_CONNECT_ATTEMPTS);
}

/****************************************/
/* Service component for MQTT           */
/* Version: 0.1                         */
/****************************************/

/*******************/
/* Includes        */
/*******************/

#include "SvcMQTT.h"      /**< Own header file */
#include <WiFi.h>         /**< WiFi library */
#include <PubSubClient.h> /**< MQTT library */

/*******************/
/* Config          */
/*******************/

#define SVCMQTT_MAX_CONNECT_ATTEMPTS   (3u)    /**< Specify max number of attempts to connect to MQTT broker */
#define SVCMQTT_CONNECT_RETRY_DELAY_MS (5000u) /**< Specify delay in ms between retries if a connect attempt has failed */

#define SVCMQTT_MAX_PUBLISH_ATTEMPTS   (3u)    /**< Specify max number of attempts to publish to MQTT */
#define SVCMQTT_PUBLISH_RETRY_DELAY_MS (5000u) /**< Specify delay in ms between retries if a publish attempt has failed */

#define SVCMQTT_PUBLISH_DELAY_MS  (100u) /**< Small delay so that the server can accept the published message */

#define SVCMQTT_1000MS (1000u)

/*******************/
/* Globals         */
/*******************/

WiFiClient(esp32Client);
PubSubClient client(esp32Client);

/*************************************/
/* Definitions of external functions */
/*************************************/

/* Function to init MQTT connection            */
/* param[in]:     domain, port, id, user, pass */
/* param[return]: isSuccess                    */
bool SvcMQTT_init(const char* sDomain, uint16_t u16Port, const char* sID, const char* sUser, const char* sPass){

    uint8_t u8ConnectAttemptCount = 0u;

    do{

      Serial.printf("Connecting to MQTT broker \"%s\" : ", sDomain);

      client.setServer(sDomain, u16Port);

      if(client.connect(sID, sUser, sPass)){
        Serial.print("OK\n");
        return true;
      } 

      // If connect attempt failed, try again
      if(u8ConnectAttemptCount == SVCMQTT_MAX_CONNECT_ATTEMPTS-1u){
        Serial.printf("FAIL, abort\n");
        return false;
      } else {
        Serial.printf("FAIL, retry in %ds...\n", SVCMQTT_CONNECT_RETRY_DELAY_MS/SVCMQTT_1000MS);
      }
      delay(SVCMQTT_CONNECT_RETRY_DELAY_MS);
      u8ConnectAttemptCount++;
      
    } while (u8ConnectAttemptCount < SVCMQTT_MAX_CONNECT_ATTEMPTS);  
}

/* Function to publish to MQTT                 */
/* param[in]:     topic, payload, isRetained   */
/* param[return]: isSuccess                    */
bool SvcMQTT_publish(const char* sTopic, const char* sPayload, bool isRetained){

    uint8_t u8PublishAttemptCount = 0u;

    do{

      Serial.printf("Publishing on MQTT topic \"%s\" : ", sTopic);

      if(client.publish(sTopic, sPayload, isRetained)){
        delay(SVCMQTT_PUBLISH_DELAY_MS); // Small delay so that the server can accept the published message
        Serial.printf("OK\n");
        return true;
      }

      // If publish attempt failed, try again
      if(u8PublishAttemptCount == SVCMQTT_MAX_PUBLISH_ATTEMPTS-1u){
        Serial.printf("FAIL, abort\n");
        return false;
      } else {
        Serial.printf("FAIL, retry in %ds...\n", SVCMQTT_PUBLISH_RETRY_DELAY_MS/SVCMQTT_1000MS);
      }
      delay(SVCMQTT_PUBLISH_RETRY_DELAY_MS);
      u8PublishAttemptCount++;
      
    } while (u8PublishAttemptCount < SVCMQTT_MAX_PUBLISH_ATTEMPTS);  
}

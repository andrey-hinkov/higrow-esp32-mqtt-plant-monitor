#ifndef SVCMQTT_H
#define SVCMQTT_H

/*******************/
/* Includes        */
/*******************/

#include <Arduino.h>

/************************************/
/* Prototypes of external functions */
/************************************/

bool SvcMQTT_init(const char* sDomain, uint16_t u16Port, const char* sID, const char* sUser, const char* sPass); /**< Function to init MQTT connection */
bool SvcMQTT_publish(const char* sTopic, const char* sPayload, bool isRetained); /**< Function to publish to MQTT */

#endif /* SVCMQTT_H */

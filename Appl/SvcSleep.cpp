/****************************************/
/* Service component for deep sleep     */
/* Version: 0.1                         */
/****************************************/

/*******************/
/* Includes        */
/*******************/

#include "SvcSleep.h" /**< Own header file */

/*******************/
/* Config          */
/*******************/

#define US_TO_S_FACTOR (1000000u) /**< Conversion factor microseconds to seconds */

/*************************************/
/* Definitions of external functions */
/*************************************/

/* Function to go to deep sleep for defined time */
/* param[in]:     TimeToSleep in seconds         */
/* param[return]: none                           */
void SvcSleep_goToSleep(uint16_t u16TimeToSleepS){
  
  // TODO: switch off power supply to sensors and any other unneccessary hardware
  // before going to sleep. Unfortunately not possibe with the current HiGrow PCB.
  
  esp_sleep_enable_timer_wakeup(u16TimeToSleepS * US_TO_S_FACTOR);

  Serial.printf("Going to sleep for %d seconds.\n", u16TimeToSleepS);
  
  esp_deep_sleep_start();
}

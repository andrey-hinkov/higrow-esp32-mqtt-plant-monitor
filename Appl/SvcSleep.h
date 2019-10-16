#ifndef SVCSLEEP_H
#define SVCSLEEP_H

/*******************/
/* Includes        */
/*******************/

#include <Arduino.h>

/************************************/
/* Prototypes of external functions */
/************************************/

void SvcSleep_goToSleep(uint16_t u16TimeToSleepS); /**< Function to go to deep sleep for defined time */

#endif /* SVCSLEEP_H */

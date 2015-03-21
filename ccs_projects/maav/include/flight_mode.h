#ifndef FLIGHT_MODE_H_
#define FLIGHT_MODE_H_

#include "rc.h"

typedef enum { AUTONOMOUS, MANUAL, ASSISTED } FLIGHT_MODE;

FLIGHT_MODE flightModeGet();

#endif /* FLIGHT_MODE_H_ */

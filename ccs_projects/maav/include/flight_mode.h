#ifndef FLIGHT_MODE_H_
#define FLIGHT_MODE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "rc.h"

typedef enum { AUTONOMOUS, MANUAL, ASSISTED } FLIGHT_MODE;

FLIGHT_MODE flightModeGet();

#ifdef __cplusplus
}
#endif

#endif /* FLIGHT_MODE_H_ */

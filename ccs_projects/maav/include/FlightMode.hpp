#ifndef FLIGHTMODE_HPP_
#define FLIGHTMODE_HPP_

#include "rc.hpp"

typedef enum { AUTONOMOUS, MANUAL, ASSISTED } FlightMode;

FlightMode flightModeGet();


#endif /* FLIGHTMODE_Hpp_ */

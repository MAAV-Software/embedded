#include <stdint.h>
#include <cstdlib>
#include <cmath>
#include "Vehicle.hpp"
#include "FlightMode.hpp"
#include "MaavMath.hpp"
//#include "servoIn.hpp"
//#include "rc.hpp"

#ifdef LINUX
#include "cmeigen.hpp"
#else
#include "time_util.h"
#include "arm_math.h"
#endif

#ifdef BENCHTOP
#include "uartstdio.h"
#endif

//added
//#ifndef PI
//#define PI 3.14159265358979f
//#endif

using namespace std;
using MaavMath::mat_at;

// define pi and gravity
#ifndef PI
static const float PI = 3.14159265358979323846;
#endif
//static const float GRAVITY = 9.81;

Dji Vehicle::getDjiVals() const
{
	return dji;
}

uint8_t Vehicle::getRCInputError()
{
    return inputerror;
}

void Vehicle::setRCInputError(uint8_t flag)
{
    inputerror |= flag;
}

void Vehicle::clearRCInputError()
{
    inputerror = 0;
}

void Vehicle::setCtrlInput(float roll, float pitch, float yaw, float thrust)
{
	dji.roll = roll;
	dji.pitch = pitch;
	dji.yawRate = yaw;
	dji.thrust = thrust;
}

// End of File

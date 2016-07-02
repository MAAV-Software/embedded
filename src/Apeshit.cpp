#include <stdint.h>
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"

#include "Apeshit.hpp"
//#include "servoIn.hpp"
//#include "rc.hpp"
//#include "PPM.h"


void goApeshit(uint32_t time)
{
    //Do, forever
    for(;;)
    {
/*
    	//Tell DJI to stop doing the things
        //PITCH
        PPM_setPulse(0, (uint32_t)map(0, -0.5, 0.5, 105600, 135000));
        //THROTTLE
        PPM_setPulse(2, (uint32_t)map(0, 0, 46.6956, 114000, 124000));
        //ROLL
        PPM_setPulse(1, (uint32_t)map(0, -0.5, 0.5, 104200, 135000));
*/
        //Play with LEDs
        Toggle_LED(RED_LED  , time);
        Toggle_LED(GREEN_LED, time);
        Toggle_LED(BLUE_LED , time);
    }
}

void goApeshit()
{
    goApeshit(APESHIT_CYCLE_TIME_DEFAULT);
}

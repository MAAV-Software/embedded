/*
 * main.c
 *
 * Main flight control program for MAAV. Iterfaces with Atom, RC controllers,
 * and DJI. Executes outerlook position control of DJI (which handles inner
 * loop attitude control).
 *
 *      Author: Sajan Patel, Jonathan Kurzer, Sasawat Prankprakma, Clark Zhang
 *        Date: Feb 24, 2015
 *
 */
#include <stdint.h>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <cmath>

#include "servoIn.hpp"

#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c123gh6pm.h"

#include "driverlib/gpio.h"
#include "driverlib/fpu.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/eeprom.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/interrupt.h"

#include "utils/uartstdio.h"

#include "PPM.h"
//#include "px4_i2c.h"
#include "time_util.h"

//#include "px4_kalman.h"
//#include "Dof.hpp"
//#include "QuadCtrl.hpp"
//#include "switch.h"
#include "rc.hpp"
#include "LED.h"
//#include "flight_mode.h"
#include "Loop.hpp"
#include "Vehicle.hpp"
#include "Dof.hpp"
#include "Pid.hpp"
#include "EEPROM.h"

//#include "tests/test_definitions.h"
#include "runnables/DjiRunnable.hpp"
#include "runnables/FlightModeRunnable.hpp"
#include "runnables/ImuRunnable.hpp"
#include "runnables/I2CRunnable.hpp"
#include "runnables/CtrlRunnable.hpp"
#include "runnables/KillRunnable.hpp"
#include "runnables/BatteryRunnable.hpp"

//#include "messaging/data_link.h"

#include "utility.h"

#include "ProgramState.hpp"

using namespace std;

bool px4_can_transmit = true;

////////////////////////////// MAIN FUNCTION ///////////////////////////////////
int main()
{
	// TODO: PUT THESE CONFIGURATIONS FOR CLOCK AND GPIO INTO APPROPRIATE
	// CONFIG FUNCTIONS!!!! EVERYONE IS FORGETTING TO TURN THE DAMN FPU ON!!!
	// IT'S THE *ONLY* REASON WE'RE USING THE TIVA IN THE FIRST PLACE!!!!
	MAP_FPULazyStackingEnable();
	MAP_FPUEnable();

	MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
				   SYSCTL_XTAL_16MHZ);
//	MAP_SysCtlClockSet(SYSCTL_SYSDIV_3 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
//					   SYSCTL_XTAL_16MHZ);

	Config_LED();

	ConfigureUART();
	UARTprintf("\nTesting Uart Connection.\n\r");

	Config_EEPROM();

	time_init(SYSCTL_PERIPH_TIMER1, SYSCLOCK, TIMER1_BASE, INT_TIMER1A);	// Chose any open timer
	PPM_init(SYSCTL_PERIPH_TIMER2, SYSCLOCK, TIMER2_BASE, INT_TIMER2A,		// Chose any open timer
			 GPIO_PORTB_BASE, GPIO_PIN_6, 4);								// Chose any open port/pin
	servoIn_init(SYSCTL_PERIPH_TIMER4, TIMER4_BASE); 						// Chose timer4 until encapsulated
	servoIn_attachPin();

	float time = (float)millis() / 1000.0f; // grab current time for initialization

	// Ctrl Vals
	// state and setpt will always be initialized the to 0 and time
	float states[4][4] = {
			{0, 0, 0, time},
			{0, 0, 0, time},
			{0, 0, 0, time},
			{0, 0, 0, time},
	};
	float setpts[4][4] = {
			{0, 0, 0, time},
			{0, 0, 0, time},
			{0, 0, 0, time},
			{0, 0, 0, time},
	};
	// intial gains will be the same for val and rate PIDs
	// TODO intialize gains from EEPROM storage of them
	float valueGains[4][3] = {
			{1, 0, 1},
			{1, 0, 1},
			{1, 0, 1},
			{1, 0, 1},
	};
	float rateGains[4][3] = {
			{1, 0, 1},
			{1, 0, 1},
			{1, 0, 1},
			{1, 0, 1},
	};
	uint8_t valueFlags[4] = {
			DERR_DT_MASK,
			DERR_DT_MASK,
			DERR_DT_MASK,
			DERR_DT_MASK | DISC_DERIV_MASK | WRAP_AROUND_MASK,
	};
	uint8_t rateFlags[4] = {
			DERR_DT_MASK | DISC_DERIV_MASK,
			DERR_DT_MASK | DISC_DERIV_MASK,
			DERR_DT_MASK | DISC_DERIV_MASK,
			DERR_DT_MASK | DISC_DERIV_MASK,
	};
	float inertias[4] = {2.5f, 2.5f, 2.5f, 2.5f};
	float stateBounds[4] = {0, 0, 0, PI};
	float rateUpLims[4] = {20, 20, 20, 20};
	float rateLwLims[4] = {-20, -20, -20, -20};
	float accelUpLims[4] = {HUGE_VALF, HUGE_VALF, HUGE_VALF, HUGE_VALF};
	float accelLwLims[4] = {-HUGE_VALF, -HUGE_VALF, -HUGE_VALF, -HUGE_VALF};

	// todo for now, lp will be set to 0 and disabled.
	float valueStateLpCoeffs[NUM_DOFS] = {0, 0, 0, 0};
	float valueErrorLpCoeffs[NUM_DOFS] = {0, 0, 0, 0};
	float rateStateLpCoeffs[NUM_DOFS] = {0, 0, 0, 0};
	float rateErrorLpCoeffs[NUM_DOFS] = {0, 0, 0, 0};

	float rpLims[NUM_ANGLES] = {PI / 4.0f, PI / 4.0f};

	// EKF values
	float ekfInitState[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
	float ekfInitP[81]; memset(ekfInitP, 0, sizeof(float) * 81);
	float ekfQ[81] = {
			1, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 1, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 1, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 1, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 1, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 1, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 1, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 1, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 1,
	};
	float ekfNoCamR[36] = {
			1, 0, 0, 0, 0, 0,
			0, 1, 0, 0, 0, 0,
			0, 0, 1, 0, 0, 0,
			0, 0, 0, 1, 0, 0,
			0, 0, 0, 0, 1, 0,
			0, 0, 0, 0, 0, 1,
	};
	float ekfWithCamR[64] = {
			1, 0, 0, 0, 0, 0, 0, 0,
			0, 1, 0, 0, 0, 0, 0, 0,
			0, 0, 1, 0, 0, 0, 0, 0,
			0, 0, 0, 1, 0, 0, 0, 0,
			0, 0, 0, 0, 1, 0, 0, 0,
			0, 0, 0, 0, 0, 1, 0, 0,
			0, 0, 0, 0, 0, 0, 1, 0,
			0, 0, 0, 0, 0, 0, 0, 1,
	};

	float totalMass = 2.5f;
	float initTime = time;
	Vehicle v(states, setpts, valueGains, rateGains, valueFlags,
			  rateFlags, inertias, stateBounds, rateUpLims,
			  rateLwLims, accelUpLims, accelLwLims,
			  valueStateLpCoeffs, valueErrorLpCoeffs,
			  rateStateLpCoeffs, rateErrorLpCoeffs,
			  totalMass, initTime, rpLims,
			  ekfInitState, ekfInitP, ekfQ, ekfNoCamR, ekfWithCamR);
	Imu imu;
	Lidar lidar;
	Px4 px4;
	SdCard sdcard;
	Battery battery;
	ProgramState pState(&v, &imu, &px4, &lidar, &sdcard, &battery, MANUAL);
	
	FlightModeRunnable flightModeRunnable(&pState);
	DjiRunnable djiRunnable(&pState);
	KillRunnable killRunnable(&pState);
	ImuRunnable	imuRunnable(&pState);
	I2CRunnable i2cRunnable(&pState);
	CtrlRunnable ctrlRunnable(&pState);
	BatteryRunnable batteryRunnable(&pState);

	Loop mainLoop;
	mainLoop.regEvent(&killRunnable, 0, 0);
	mainLoop.regEvent(&flightModeRunnable, 10, 1);
	mainLoop.regEvent(&ctrlRunnable, 10, 4);
	mainLoop.regEvent(&djiRunnable, 10, 5);
	mainLoop.regEvent(&imuRunnable, 0, 2);
	mainLoop.regEvent(&i2cRunnable, 0, 3);
	mainLoop.regEvent(&batteryRunnable, 1000, 6);

	// tricky way to get rid of the initial large values!
	while(servoIn_getPulse(KILL_CHAN3) > 120000);

	// check if the stick is up, PPM range(59660, 127400)
	// might change after the calibration (87552, 153108)
	while(servoIn_getPulse(KILL_CHAN3) < 120000);

	sdcard.createFile();

	mainLoop.run();

	return 0;
}
// End of File


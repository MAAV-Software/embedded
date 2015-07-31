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

#include "servoIn.hpp"
#include "switch.h"
#include "utility.h"
#include "PPM.h"
#include "time_util.h"
#include "rc.hpp"
#include "LED.h"
#include "Loop.hpp"
#include "Vehicle.hpp"
#include "Dof.hpp"
#include "Pid.hpp"
#include "messaging/DataLink.hpp"
#include "DataLinkHw.hpp"
#include "ProgramState.hpp"

#include "runnables/DataLinkRunnable.hpp"
#include "runnables/DjiRunnable.hpp"
#include "runnables/FlightModeRunnable.hpp"
#include "runnables/ImuRunnable.hpp"
#include "runnables/I2CRunnable.hpp"
#include "runnables/CtrlRunnable.hpp"
#include "runnables/KillRunnable.hpp"
#include "runnables/SwitchRunnable.hpp"

using namespace std;

////////////////////////////// MAIN FUNCTION ///////////////////////////////////
int main()
{
	MAP_FPULazyStackingEnable();
	MAP_FPUEnable();

	MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
				   SYSCTL_XTAL_16MHZ);

	Config_LED();
	time_init(SYSCTL_PERIPH_TIMER1, SYSCLOCK, TIMER1_BASE, INT_TIMER1A);	// Chose any open timer
	PPM_init(SYSCTL_PERIPH_TIMER2, SYSCLOCK, TIMER2_BASE, INT_TIMER2A,		// Chose any open timer
			 GPIO_PORTB_BASE, GPIO_PIN_6, 4);								// Chose any open port/pin
	servoIn_init(SYSCTL_PERIPH_TIMER4, TIMER4_BASE); 						// Chose timer4 until encapsulated
	servoIn_attachPin();

	// todo read in gains from eeprom here
	float valueGains[NUM_DOFS][NUM_PID_GAINS] = {
		{1, 0, 0},
		{1, 0, 0},
		{1, 0, 0},
		{1, 0, 0}
	};
	float rateGains[NUM_DOFS][NUM_PID_GAINS] = {
		{1, 0, 0},
		{1, 0, 0},
		{1, 0, 0},
		{0, 0, 0}
	};

	Vehicle v(valueGains, rateGains);
	Imu imu;
	Lidar lidar;
	Px4 px4;
	DataLink dl(DataLinkUartSend);
	SdCard sdcard;
	SwitchData_t sw[3];
	ProgramState pState(&v, &imu, &px4, &lidar, &sdcard, MANUAL, &dl, sw);
	
	// Constructing Runnables also initializes the hardware for them
	FlightModeRunnable flightModeRunnable(&pState);
	DjiRunnable djiRunnable(&pState);
	KillRunnable killRunnable(&pState);
	ImuRunnable	imuRunnable(&pState);
	I2CRunnable i2cRunnable(&pState);
	CtrlRunnable ctrlRunnable(&pState);
	SwitchRunnable switchRunnable(&pState);
	DataLinkRunnable dlinkRunnable(&pState);

	Loop mainLoop;
	mainLoop.regEvent(&killRunnable, 		0, 		0);
	mainLoop.regEvent(&flightModeRunnable, 	10, 	1);
	mainLoop.regEvent(&imuRunnable, 		0, 		2);
	mainLoop.regEvent(&i2cRunnable, 		0, 		3);
	mainLoop.regEvent(&ctrlRunnable, 		10, 	4);
	mainLoop.regEvent(&djiRunnable, 		10, 	5);
	mainLoop.regEvent(&dlinkRunnable, 		20, 	6);
	mainLoop.regEvent(&switchRunnable, 		50, 	7);

	// tricky way to get rid of the initial large values!
	while(servoIn_getPulse(KILL_CHAN3) > 120000);

	// check if the stick is up, PPM range(59660, 127400)
	// might change after the calibration
	while(servoIn_getPulse(KILL_CHAN3) < 120000);

	sdcard.createFile("log0.txt");

//	char buf[100];
//	for (uint32_t i = 0; i < 50000; ++i)
//	{
//		uint32_t len = snprintf(buf, sizeof(buf), "Servo: %u\n\n", servoIn_getPulse(KILL_CHAN3));
//		sdcard.write(buf, len);
//	}
//	sdcard.closeFile();

	mainLoop.run();

	return 0;
}
// End of File


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
//#include "led.h"
//#include "flight_mode.h"
#include "Loop.hpp"
#include "Vehicle.hpp"
//#include "tests/test_definitions.h"
#include "runnables/DjiRunnable.hpp"
#include "runnables/FlightModeRunnable.hpp"
#include "runnables/ImuRunnable.hpp"
#include "runnables/I2CRunnable.hpp"

//#include "messaging/data_link.h"

#include "utility.h"

#include "ProgramState.hpp"

bool px4_can_transmit = true;

////////////////////////////// MAIN FUNCTION ///////////////////////////////////
int main()
{
	// TODO: PUT THESE CONFIGURATIONS FOR CLOCK AND GPIO INTO APPROPRIATE
	// CONFIG FUNCTIONS!!!! EVERYONE IS FORGETTING TO TURN THE DAMN FPU ON!!!
	// IT'S THE *ONLY* REASON WE'RE USING THE TIVA IN THE FIRST PLACE!!!!
	FPULazyStackingEnable();
	FPUEnable();

//	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
//				   SYSCTL_XTAL_16MHZ);
	SysCtlClockSet(SYSCTL_SYSDIV_3 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
					   SYSCTL_XTAL_16MHZ);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);

	ConfigureUART();
	UARTprintf("Testing Uart Connection.\n\r");

	time_init(SYSCTL_PERIPH_TIMER1, SYSCLOCK, TIMER1_BASE, INT_TIMER1A);	// Chose any open timer
	PPM_init(SYSCTL_PERIPH_TIMER2, SYSCLOCK, TIMER2_BASE, INT_TIMER2A,		// Chose any open timer
			 GPIO_PORTB_BASE, GPIO_PIN_6, 4);								// Chose any open port/pin
	servoIn_init(SYSCTL_PERIPH_TIMER4, TIMER4_BASE); // Chose timer4 until encapsulated
	servoIn_attachPin();

	Vehicle vehicle;
	Imu imu;
	Lidar lidar;
	Px4 px4;
	ProgramState pState(&vehicle, &imu, &px4, &lidar, MANUAL);
	
	FlightModeRunnable flightModeRunnable(&pState);
	DjiRunnable djiRunnable(&pState);
	ImuRunnable	imuRunnable(&pState);
	I2CRunnable i2cRunnable(&pState);

	Loop mainLoop;
	mainLoop.addEvent(&flightModeRunnable, 10);
	mainLoop.addEvent(&djiRunnable, 10);
	mainLoop.addEvent(&imuRunnable, 0);
	mainLoop.addEvent(&i2cRunnable, 0);
	mainLoop.run();

	return 0;
}
// End of File

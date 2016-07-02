/*
 * main.c
 *
 * Main flight control program for MAAV. Iterfaces with Atom, RC controllers,
 * and DJI. Executes outerlook position control of DJI (which handles inner
 * loop attitude control).
 *
 *      Author: Sajan Patel, Zhengjie Cui, Jonathan Kurzer, Sasawat Prankprakma, Clark Zhang
 *        Date: Feb 15, 2016
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

//#include "time_util.h"
#include "EEPROM.h"
#include "SdCard.hpp"
#include "Apeshit.hpp"

using namespace std;

////////////////////////////// MAIN FUNCTION ///////////////////////////////////
int main()
{
	// init fpu, clock, LEDs, EEPROM, and timer
	MAP_FPULazyStackingEnable();
	MAP_FPUEnable();
	//MAP_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
					   SYSCTL_XTAL_16MHZ);

//	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	// Enable the GPIO pins for the LED (PF1 & PF2 & PF3).
//	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0);

	Config_LED();

	Config_EEPROM();
	//time_init(SYSCTL_PERIPH_TIMER1, SYSCLOCK, TIMER1_BASE, INT_TIMER1A);	// Chose any open timer

	// construct and mount SD card and open a file on it
	SdCard sdcard;
	sdcard.createFile();

	// grab data for message
//	uint32_t msTime = millis();
	float time = 1.20; //((float)msTime) / 1000.0f; // grab current time

	// format message and write it
	char msg[1024];
	uint32_t len = snprintf(msg, sizeof(msg), "FUCK TAN AND SASAWAT, THE RETARD\nThe time is %f", time);
	sdcard.write(msg, len);

	// sync and close file
	sdcard.sync();
	sdcard.closeFile();

/*
	for (uint8_t i = 0; i < 10; ++i)
	{
		MAP_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0); // LOW
		MAP_SysCtlDelay(5 * APESHIT_CYCLE_TIME_DEFAULT);
		MAP_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1); // HI
		MAP_SysCtlDelay(5 * APESHIT_CYCLE_TIME_DEFAULT);
	}
*/
	for (;;) Toggle_LED(GREEN_LED, 5 * APESHIT_CYCLE_TIME_DEFAULT);

	//return 0;
}
// End of File

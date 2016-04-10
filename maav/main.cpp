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

#include "utils/uartstdio.h"

#include "servoIn.hpp"
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
#include "EEPROM.h"
#include "RcController.hpp"
#include "RcProfiles.hpp"
#include "kalman/KalmanFilter.hpp"
#include "Switch.hpp"

#include "runnables/DataLinkRunnable.hpp"
#include "runnables/DjiRunnable.hpp"
#include "runnables/FlightModeRunnable.hpp"
#include "runnables/ImuRunnable.hpp"
#include "runnables/I2CRunnable.hpp"
#include "runnables/CtrlRunnable.hpp"
#include "runnables/KillRunnable.hpp"
#include "runnables/SwitchRunnable.hpp"
#include "runnables/BatteryRunnable.hpp"

using namespace std;

////////////////////////////// MAIN FUNCTION ///////////////////////////////////
int main()
{

	MAP_FPULazyStackingEnable();
	MAP_FPUEnable();

	MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
					   SYSCTL_XTAL_16MHZ);

	Config_LED();
	Config_EEPROM();
	time_init(SYSCTL_PERIPH_TIMER1, SYSCLOCK, TIMER1_BASE, INT_TIMER1A);	// Chose any open timer
	PPM_init(SYSCTL_PERIPH_TIMER2, SYSCLOCK, TIMER2_BASE, INT_TIMER2A,		// Chose any open timer
			 GPIO_PORTB_BASE, GPIO_PIN_6, 4);								// Chose any open port/pin
	servoIn_init(SYSCTL_PERIPH_TIMER4, TIMER4_BASE); 						// Chose timer4 until encapsulated
	servoIn_attachPin();

	//Initialize Switches
	ThreeSwitch sw[3] =
	{
	        ThreeSwitch(SYSCTL_PERIPH_GPIOF, GPIO_PORTF_BASE, GPIO_PIN_4),
	        ThreeSwitch(SYSCTL_PERIPH_GPIOB, GPIO_PORTB_BASE, GPIO_PIN_0),
	        ThreeSwitch(SYSCTL_PERIPH_GPIOB, GPIO_PORTB_BASE, GPIO_PIN_1)
	};

	float valueGains[NUM_DOFS][NUM_PID_GAINS];
	float rateGains[NUM_DOFS][NUM_PID_GAINS];
	float pidEeprom[NUM_FLOAT];
	Read_PID_EEPROM(pidEeprom);

	valueGains[X_AXIS][KP] = pidEeprom[0];
	valueGains[X_AXIS][KI] = pidEeprom[1];
	valueGains[X_AXIS][KD] = pidEeprom[2];
    valueGains[Y_AXIS][KP] = pidEeprom[3];
    valueGains[Y_AXIS][KI] = pidEeprom[4];
    valueGains[Y_AXIS][KD] = pidEeprom[5];
    valueGains[Z_AXIS][KP] = pidEeprom[6];
    valueGains[Z_AXIS][KI] = pidEeprom[7];
    valueGains[Z_AXIS][KD] = pidEeprom[8];
    valueGains[YAW][KP]    = pidEeprom[9];
    valueGains[YAW][KI]    = pidEeprom[10];
    valueGains[YAW][KD]    = pidEeprom[11];
    rateGains[X_AXIS][KP]  = pidEeprom[12];
    rateGains[X_AXIS][KI]  = pidEeprom[13];
    rateGains[X_AXIS][KD]  = pidEeprom[14];
    rateGains[Y_AXIS][KP]  = pidEeprom[15];
    rateGains[Y_AXIS][KI]  = pidEeprom[16];
    rateGains[Y_AXIS][KD]  = pidEeprom[17];
    rateGains[Z_AXIS][KP]  = pidEeprom[18];
    rateGains[Z_AXIS][KI]  = pidEeprom[19];
    rateGains[Z_AXIS][KD]  = pidEeprom[20];
    rateGains[YAW][KP]     = pidEeprom[21];
    rateGains[YAW][KI]     = pidEeprom[22];
    rateGains[YAW][KD]     = pidEeprom[23];

//	valueGains[X_AXIS][KP] = 0.1;
//	valueGains[X_AXIS][KI] = 0;
//	valueGains[X_AXIS][KD] = 0;
//    valueGains[Y_AXIS][KP] = 0.1;
//    valueGains[Y_AXIS][KI] = 0;
//    valueGains[Y_AXIS][KD] = 0;
//    valueGains[Z_AXIS][KP] = 5;
//    valueGains[Z_AXIS][KI] = 0.2;
//    valueGains[Z_AXIS][KD] = 0.3;
//    valueGains[YAW][KP]    = 0.1;
//    valueGains[YAW][KI]    = 0;
//    valueGains[YAW][KD]    = 0;
//    rateGains[X_AXIS][KP]  = 0.1;
//    rateGains[X_AXIS][KI]  = 0;
//    rateGains[X_AXIS][KD]  = 0;
//    rateGains[Y_AXIS][KP]  = 0.1;
//    rateGains[Y_AXIS][KI]  = 0;
//    rateGains[Y_AXIS][KD]  = 0;
//    rateGains[Z_AXIS][KP]  = 0.01;
//    rateGains[Z_AXIS][KI]  = 0.1;
//    rateGains[Z_AXIS][KD]  = 0.001;
//    rateGains[YAW][KP]     = 0.1;
//    rateGains[YAW][KI]     = 0;
//    rateGains[YAW][KD]     = 0;

	Vehicle v(valueGains, rateGains);
	Imu imu;
	Lidar lidar;
	Px4 px4;
	DataLink dl(DataLinkUartSend);
	SdCard sdcard;

	Battery battery;
	feedback_t fbMsg;

	RcController kill(Maav::hitec, Maav::hitecNumch);
	RcController pilot(Maav::futaba, Maav::futabaNumch);
	RcOutput djiout(Maav::dji, Maav::djiNumch);

	ProgramState pState(&v, &imu, &px4, &lidar, &sdcard, &battery, MANUAL,
	        &dl, sw, &fbMsg, &kill, &pilot, &djiout);
	
	// Constructing Runnables also initializes the hardware for them
	FlightModeRunnable flightModeRunnable(&pState);
	DjiRunnable djiRunnable(&pState);
	KillRunnable killRunnable(&pState);
	ImuRunnable	imuRunnable(&pState);
	I2CRunnable i2cRunnable(&pState);
	CtrlRunnable ctrlRunnable(&pState);
	SwitchRunnable switchRunnable(&pState);
	DataLinkRunnable dlinkRunnable(&pState);
	BatteryRunnable batteryRunnable(&pState);

	Loop mainLoop;
	mainLoop.regEvent(&killRunnable, 		0, 		0);
	mainLoop.regEvent(&flightModeRunnable, 	10, 	1);
	mainLoop.regEvent(&imuRunnable, 		0, 		2);
	mainLoop.regEvent(&i2cRunnable, 		0, 		3);

	mainLoop.regEvent(&ctrlRunnable, 		10, 	4);
	mainLoop.regEvent(&djiRunnable, 		10, 	5);

	mainLoop.regEvent(&dlinkRunnable, 		20, 	6);
	mainLoop.regEvent(&switchRunnable, 		50, 	7);
	mainLoop.regEvent(&batteryRunnable, 	1000, 	8);

	// tricky way to get rid of the initial large values!
	while (kill.dutyCycle(servoIn_getPulse(KILL_CHAN3), 2) > 0.75);

	// check if the stick is up, PPM range(59660, 127400)
	// might change after the calibration (87552, 153108)
	while (kill.dutyCycle(servoIn_getPulse(KILL_CHAN3), 2) < 0.75);

/*
	while (!sw[2].readState)
	{
		switchesUpdate(sw);
	}
*/
	//The first unkill the kill runnable isn't running yet
	//So we have to manually do the unkill tasks
	killRunnable.resetYaw();
	sdcard.createFile();

//	char buf[100];
//	while (!sw[2].readState)
//	{
//		uint32_t len = snprintf(buf, sizeof(buf), "%u\t%u\t%u\t%u\t%u\n",
//				servoIn_getPulse(RC_CHAN1),
//			    servoIn_getPulse(RC_CHAN2),
//			    servoIn_getPulse(RC_CHAN3),
//			    servoIn_getPulse(RC_CHAN4),
//			    millis());
//		sdcard.write(buf, len);
//
//		switchesUpdate(sw);
//	}
//
//	sdcard.closeFile();

	emergency_t ems;
	ems.status = (int8_t)EMERGENCY_T_NORMAL;
	dl.send(&ems);

#ifdef BENCHTOP
	UARTStdioConfig(0, 115200, 16000000);
	UARTprintf("Starting MAAV Ctrls Main Loop\n");
#endif

//	for (int i = 0; i < 6; ++i)
//		Toggle_LED(BLUE_LED, SYSCLOCK / 3 / 2);
//
//
//	uint32_t curTime = millis();
//	while ((millis() - curTime) < 2000)
//	{
//		PPM_setPulse(0, 153372);    // Chan 1 - max
//		PPM_setPulse(1, 86588);    	// Chan 2 - min
//		PPM_setPulse(2, 71602);		// Chan 3 - min
//		PPM_setPulse(3, 152888);    // Chan 4 - max
//	}
//
//	PPM_setPulse(0, 120364);    // Chan 1 - mid
//	PPM_setPulse(1, 119735);    // Chan 2 - mid
//	PPM_setPulse(2, 120300);    	// Chan 3
//	PPM_setPulse(3, 119784);    // Chan 4 - mid

	mainLoop.run();

	return 0;
}
// End of File

/*
 * I2CRunnable.cpp
 *
 *  Created on: Jul 10, 2015
 *      Author: Zhengjie
 */

#include "runnables/I2CRunnable.hpp"
#include "I2CHw.hpp"

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "sensorlib/i2cm_drv.h"

#include "utils/uartstdio.h"

#include "time_util.h"
#include <cstring>

using namespace std;

I2CRunnable::I2CRunnable(ProgramState *pState) : ps(pState)
{
//	ConfigI2C(SYSCTL_PERIPH_I2C3, SYSCTL_PERIPH_GPIOD, GPIO_PD0_I2C3SCL,
//			  GPIO_PD1_I2C3SDA, GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_1,
//			  I2C3_BASE, INT_I2C3, true); // this should move to main before the loop
	// THESE ARE GOOD FOR BOTH 2015 AND 2016 SIGNAL BOARDS
	ConfigI2C(SYSCTL_PERIPH_I2C0, SYSCTL_PERIPH_GPIOB, GPIO_PB2_I2C0SCL,
				  GPIO_PB3_I2C0SDA, GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_PIN_3,
				  I2C0_BASE, INT_I2C0, true); // this should move to main before the loop
	
	// Sending first Lidar Command
	I2CMWrite(&I2CMInst, LIDAR_I2C_ADDRESS, command, 2, I2CMCallback, 0); // Start the measurment
	currentState = Lidar_1;
}

void I2CRunnable::run(void)
{
	uint32_t getTime = MAP_TimerValueGet(TIMER4_BASE, TIMER_A);
	float sysClock = (float)SYSCLOCK;
	float time;

	if (I2CMDone)
	{
		switch(currentState)
		{
			case Lidar_1: // have sent the 1st Lidar command, ready to send px4 command
				if ((getTime - PX4Time) > (sysClock / 1000.0 * 1.5))// wait for 1.5ms after px4 done
				{
					PX4Time = getTime; // the time of px4 start
					I2CMDone = false;
					I2CMRead(&I2CMInst, PX4_I2C_ADDRESS, command + 3, 1, rawPx4, (uint_fast16_t)sizeof(px4Frame), I2CMCallback, 0);
					nextState = PX4_1;
				}
				break;
			case PX4_1: // have sent px4 command, ready to parse px4 and send 2nd lidar command
				if ((getTime - LidarTime) > (sysClock / 1000.0 * 20.0)) // wait for 20ms after lidar1 done
				{
					time = (float)millis() / 1000.0f;
					ps->px4->RecordTime(time);
					ps->px4->parse(rawPx4);

					LidarTime = getTime; // the time of lidar2 start
					I2CMDone = false;
					I2CMRead(&I2CMInst, LIDAR_I2C_ADDRESS, command + 2, 1, rawLidar, LIDAR_DIST_SIZE, I2CMCallback, 0);
					nextState = Lidar_2;
				}
				break;
			case Lidar_2: // have sent the 2nd lidar command, ready to parse lidar and send 1st lidar command
				if ((getTime - LidarTime) > (sysClock / 1000.0 * 0.1)) //wait for 0.1ms after lidar2 done
				{
					time = (float)millis() / 1000.0f;
					ps->lidar->RecordTime(time);
					ps->lidar->parse(rawLidar, LIDAR_DIST_SIZE);

					LidarTime = getTime; // the time of lidar start
					I2CMDone = false;
					I2CMWrite(&I2CMInst, LIDAR_I2C_ADDRESS, command, 2, I2CMCallback, 0);
					nextState = Lidar_1;
				}
				break;
		}
	
		currentState = nextState;
	}
}

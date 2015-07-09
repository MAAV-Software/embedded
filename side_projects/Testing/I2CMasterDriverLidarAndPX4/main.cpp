/*
 * main.cpp
 *
 *  Created on: Jun 24, 2015
 *      Author: Zhengjie
 */
#include "I2C_defines.hpp"
#include "I2C_hw.hpp"
#include "Lidar.hpp"
#include "PX4.hpp"


int main(void){
	PX4 px4;
	Lidar lidar;

    // Setup the system clock to run at 50 MHz from PLL with crystal reference
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

	ConfigUART();

    // Print the welcome message to the terminal.
    UARTprintf("I2CMaster Driver PX4 & Lidar Testing\n\r");

	ConfigI2C();

	TimerInit();

	// Initialize the I2C master driver. It is assumed that the I2C module has
	// already been enabled and the I2C pins have been configured.
	I2CMInit(&g_sI2CMInst, I2C3_BASE, INT_I2C3, 0xff, 0xff, SysCtlClockGet());

	// Sending first Lidar Command
	g_bI2CMDone = true;
	I2CMWrite(&g_sI2CMInst, LIDAR_I2C_ADDRESS, Command, 2, I2CMCallback, 0); // Start the measurment
	while(!g_bI2CMDone){};
	currentstate = Lidar_1;

	PX4Time = 0;
	LidarTime = 0;

	while(1)
	{
		uint32_t gettime = TimerValueGet(TIMER0_BASE, TIMER_A);

		if (g_bI2CMDone)
		{
			switch(currentstate){
			case Lidar_1:
				if (gettime - PX4Time > 50000)// wait for 1ms after px4 done
				{
					PX4Time = gettime; // the time of px4 start
					g_bI2CMDone = false;
					I2CMRead(&g_sI2CMInst, PX4_I2C_ADDRESS, Command + 3, 1, px4.get_raw_in(), 22, I2CMCallback, 0);
					while(!g_bI2CMDone){};
					px4.parse();
					UARTprintf("Ground Dis(mm) PX4:%u\t", px4.getHeight());
					nextstate = PX4_1;
				}
				break;
			case PX4_1:
				if (gettime - LidarTime > 750000) // wait for 15ms after lidar1 done
				{
					LidarTime = gettime; // the time of lidar2 start
					g_bI2CMDone = false;
					I2CMRead(&g_sI2CMInst, LIDAR_I2C_ADDRESS, Command + 2, 1, lidar.get_raw_in(), 2, I2CMCallback, 0);
					while(!g_bI2CMDone){};
					lidar.parse();
					nextstate = Lidar_2;
					UARTprintf("Ground Dis(cm) Lidar:%u\n", lidar.getDistance());
				}
				break;
			case Lidar_2:
				if (gettime - LidarTime > 5000) //wait for 0.1ms after lidar2 done
				{
					LidarTime = gettime; // the time of lidar start
					g_bI2CMDone = false;
					I2CMWrite(&g_sI2CMInst, LIDAR_I2C_ADDRESS, Command, 2, I2CMCallback, 0);
					while(!g_bI2CMDone){};
					nextstate = Lidar_1;
				}
				break;
			}
			currentstate = nextstate;
		}

	}
}

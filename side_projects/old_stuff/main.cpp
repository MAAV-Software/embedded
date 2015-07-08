/*
 * main.cpp
 *
 *  Created on: Jun 24, 2015
 *      Author: Zhengjie
 */
#include "I2C_hw.hpp"
#include "Lidar.hpp"
#include "Px4.hpp"

int main(void)
{
	Px4 px4;
	Lidar lidar;
	uint8_t rawpx4[sizeof(px4Frame)];
	uint8_t rawlidar[LIDAR_DIST_SIZE];

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
//	while(!g_bI2CMDone){};
	currentstate = Lidar_1;

	PX4Time = 0;
	LidarTime = 0;

	while(1)
	{
		uint32_t gettime = TimerValueGet(TIMER0_BASE, TIMER_A);

		if (g_bI2CMDone)
		{
			switch(currentstate){
			case Lidar_1: // have sent the 1st Lidar command, ready to send px4 command
				if (gettime - PX4Time > 50000*1.5)// wait for 1.5ms after px4 done
				{
					PX4Time = gettime; // the time of px4 start
					g_bI2CMDone = false;
					I2CMRead(&g_sI2CMInst, PX4_I2C_ADDRESS, Command + 3, 1, rawpx4, sizeof(px4Frame), I2CMCallback, 0);
//					while(!g_bI2CMDone){};
					nextstate = PX4_1;
				}
				break;
			case PX4_1: // have sent px4 command, ready to parse px4 and send 2nd lidar command
				if (gettime - LidarTime > 50000*18) // wait for 18ms after lidar1 done
				{
					px4.parse(rawpx4);
					UARTprintf("Ground Dis(mm) PX4:%u\t", (uint16_t)(px4.getZDist()*1000));

					LidarTime = gettime; // the time of lidar2 start
					g_bI2CMDone = false;
					I2CMRead(&g_sI2CMInst, LIDAR_I2C_ADDRESS, Command + 2, 1, rawlidar, LIDAR_DIST_SIZE, I2CMCallback, 0);
//					while(!g_bI2CMDone){};
					nextstate = Lidar_2;
				}
				break;
			case Lidar_2: // have sent the 2nd lidar command, ready to parse lidar and send 1st lidar command
				if (gettime - LidarTime > 50000/10) //wait for 0.1ms after lidar2 done
				{
					lidar.parse(rawlidar, LIDAR_DIST_SIZE);

					uint16_t dist = (((uint16_t)rawlidar[0]) << 8) | rawlidar[1];

					UARTprintf("Ground Dis(mm) Lidar:%u\n", dist);

					LidarTime = gettime; // the time of lidar start
					g_bI2CMDone = false;
					I2CMWrite(&g_sI2CMInst, LIDAR_I2C_ADDRESS, Command, 2, I2CMCallback, 0);
//					while(!g_bI2CMDone){};
					nextstate = Lidar_1;
				}
				break;
			}
			currentstate = nextstate;
		}

	}
}

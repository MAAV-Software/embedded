/*
 * I2CHw.cpp
 *
 *  Created on: Jun 24, 2015
 *      Author: Zhengjie
 */
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/i2c.h"
#include "driverlib/timer.h"
#include "utils/uartstdio.h"

#include "Px4Defines.hpp"
#include "LidarDefines.hpp"
#include "I2CDefines.hpp"
#include "I2CHw.hpp"

uint8_t command[4] = {LIDAR_REG_MEA, LIDAR_MEA_VALE, LIDAR_FULL_BYTE, PX4_MEA};
bool I2CMDone = true;
tI2CMInstance I2CMInst;
uint32_t LidarTime = 0;
uint32_t PX4Time = 0;
uint8_t rawpx4[sizeof(px4Frame)];
uint8_t rawlidar[LIDAR_DIST_SIZE];
uint32_t I2C_BASE;

void I2CMasterIntHandler();

//******************************************************************************
void ConfigI2C(const uint32_t sysctlPeriphI2C,
			   const uint32_t sysctlPeriphGPIO,
			   const uint32_t sclPinConfig,
			   const uint32_t sdaPinConfig,
			   const uint32_t gpioBase,
			   const uint32_t gpioSclPin,
			   const uint32_t gpioSdaPin,
			   const uint32_t _i2cBase,
			   const uint32_t i2cInterrupt,
			   const bool useFastBus)
{
	i2cBase = _i2cBase;
	
	// configure peripheral ctl
    SysCtlPeripheralEnable(sysctlPeriphI2C);
    SysCtlPeripheralEnable(sysctlPeriphGPIO);

	// configure pin muxing	
    GPIOPinConfigure(sclPinConfig);
    GPIOPinConfigure(sdaPinConfig);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(gpioBase, gpioSclPin);
    GPIOPinTypeI2C(gpioBase, gpioSdaPin);

    // Initializes operation of the I2C Master block by configuring the bus speed(true for 400Kps)
    I2CMasterInitExpClk(i2cBase, SysCtlClockGet(), useFastBus);

	// register handler and enable interrupt
    I2CIntRegister(i2cBase, I2CMasterIntHandler);
    IntMasterEnable();
    
	// delay for initialization
	for (int i = 0; i < 3; ++i) SysCtlDelay(SysCtlClockGet() / 3);
	
	// Initialize the I2C master driver. It is assumed that the I2C module has
	// already been enabled and the I2C pins have been configured.
	I2CMInit(&I2CMInst, i2cBase, i2cInterrupt, 0xff, 0xff, SysCtlClockGet());
	I2CMDone = false;
}

// The interrupt handler for the I2C module.
void I2CMasterIntHandler()
{
	// Call the I2C master driver interrupt handler.
	I2CMIntHandler(&I2CMInst);
}

// The function that is provided by this Run as a callback when I2C transactions have completed.
void I2CMCallback(void *pvData, uint_fast8_t ui8Status)
{
	/* TODO: ADD LED INDICATOR
	// See if an error occurred.
	if(ui8Status != I2CM_STATUS_SUCCESS)
	{
		// An error occurred, so handle it here if required.
		UARTprintf("Error\n");
	}
	*/

	// Indicate that the I2C transaction has completed.
	I2CMDone = true;
}

// End of File

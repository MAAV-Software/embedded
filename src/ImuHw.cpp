/*
 * ImuHw.cpp
 *
 *  Created on: Jul 10, 2015
 *      Author: Zhengjie
 */

#include <stdint.h>
#include <string.h>

#include "ImuHw.hpp"
#include "ImuDefines.hpp"
#include "Apeshit.hpp"
#include "time_util.h"
#include "LED.h"

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"

using std::memcpy;
using namespace MaavImu;

// externed globals
bool IMU_DONE = false;
uint8_t IMU_RAW_DATA[MAX_IMU_DATA_LENGTH];

// file-scope globals
uint8_t IMU_RAW_IN[MAX_IMU_DATA_LENGTH];
uint8_t IMU_IDX = 0;
uint8_t IMU_CMD = 0xCC;
uint32_t UART_BASE = UART1_BASE;

//****************************************************************************************
void imuUartConfig(const uint32_t sysctlPeriphUart,
				   const uint32_t sysctlPeriphGPIO,
				   const uint32_t rxPinConfig,
				   const uint32_t txPinConfig,
				   const uint32_t gpioPortBase,
				   const uint32_t gpioRxPin,
				   const uint32_t gpioTxPin,
				   const uint32_t _uartBase,
				   const uint32_t uartInterrupt)
{
	UART_BASE = _uartBase;
    
	// Enable the GPIO Peripheral used by the UART1 on PC. UART0 on PA.
    MAP_SysCtlPeripheralEnable(sysctlPeriphGPIO);
    MAP_SysCtlPeripheralEnable(sysctlPeriphUart);

    // Configure GPIO Pins for UART mode.
    MAP_GPIOPinConfigure(rxPinConfig);
    MAP_GPIOPinConfigure(txPinConfig);
    MAP_GPIOPinTypeUART(gpioPortBase, gpioRxPin | gpioTxPin);

	// Configure UART Clock.
	MAP_UARTClockSourceSet(UART_BASE, UART_CLOCK_SYSTEM);

	//Configure UART for operation in the specified data format.
    MAP_UARTConfigSetExpClk(UART_BASE, SYSCLOCK, 115200,
			(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    MAP_UARTEnable(UART_BASE);

    // Register and Enable UART1 RX Interrupt.
    UARTIntRegister(UART_BASE, imuUartIntHandler);
    MAP_IntMasterEnable();
    MAP_IntEnable(uartInterrupt);
    MAP_UARTIntEnable(UART_BASE, UART_INT_RX | UART_INT_RT);
    
	// delay for initialization
	for (int i = 0; i < 3; ++i) Toggle_LED(BLUE_LED, SYSCLOCK / 3 / 2);
    //MAP_SysCtlDelay(SYSCLOCK / 3);
	
	IMU_DONE = false;
}

//****************************************************************************************
void imuUartIntHandler()
{
	//Check flags and based on those flags set a calibration state
	uint32_t length = 0;
    switch (IMU_CMD)
    {
    	case MEASUREMENT_CMD:   length = MEASUREMENT_DATA_LENGTH;   break;
    	case ACCEL_CALIB_CMD:   length = ACCEL_BIAS_DATA_LENGTH;    break;
    	case GYRO_CALIB_CMD:    length = GYRO_BIAS_DATA_LENGTH;     break;
        default: goApeshit(); break;
    }
    
    uint32_t status = MAP_UARTIntStatus(UART_BASE, true);

    // Clear the asserted interrupts.
    MAP_UARTIntClear(UART_BASE, status);

    // Loop while there are characters in the receive FIFO.
    while (MAP_UARTCharsAvail(UART_BASE))
    {
        // Read next data byte
        uint8_t data = MAP_UARTCharGetNonBlocking(UART_BASE);

        if ((data == IMU_CMD) 
                && ((IMU_IDX == 0) || (IMU_IDX >= (uint8_t)length)))
        {
        	IMU_IDX = 0;
        	IMU_DONE = false;
        }

        IMU_RAW_IN[IMU_IDX++] = data;

		if (IMU_IDX >= (uint8_t)length)
		{
			//imuMemcpy(imuRawFinal, imuRawIn, (int)length);
			memcpy(IMU_RAW_DATA, IMU_RAW_IN, length);
            IMU_DONE = true;
		}
	}
}

//****************************************************************************************
void imuUartSend(const uint8_t *buffer, const uint32_t count)
{
    if ((buffer == NULL) && (count < 1)) return;
    
    IMU_CMD = buffer[0]; // first byte is always the cmd
    
    // Loop while there are more characters to send
    for (int i = 0; i < count; ++i) MAP_UARTCharPut(UART_BASE, *buffer++);
}

/*
// TODO May want to rename as MemCpy and move to MAAV utility library
void imuMemcpy(uint8_t* out, const uint8_t* in, const int len)
{
	for (int i = 0; i < len; i++) out[i] = in[i];
}
*/

// End of File

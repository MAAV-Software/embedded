/*
 * utility.c
 *
 * Implementation of general utility library for MAAV controls.
 *
 *  Created on: Dec 18, 2014
 *      Author: Sajan Patel, Jonathan Kurzer
 */
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c123gh6pm.h"

#include "driverlib/gpio.h"
#include "driverlib/fpu.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

#include "utils/uartstdio.h"

#include "PPM.h"
#include "time_util.h"

/****************** Serial Port/Kalman Debug Utility Functions ****************/
//void sendToSerialPort(kalman_t* filter, uint16_t frameCount)
//{
//	char buffer[150];
//	uint32_t len = snprintf(buffer, 150,
//			"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
//			frameCount,
//			px4_i2c_get_pixel_flow_x_sum(),
//			px4_i2c_get_pixel_flow_y_sum(),
//			px4_i2c_get_flow_comp_m_x(),
//			px4_i2c_get_flow_comp_m_y(),
//			px4_i2c_get_qual(),
//			px4_i2c_get_gyro_x_rate(),
//			px4_i2c_get_gyro_y_rate(),
//			px4_i2c_get_gyro_z_rate(),
//			px4_i2c_get_gyro_range(),
//			px4_i2c_getTimestep(),
//			px4_i2c_getHeight(),
//			filter->xdot,
//			filter->ydot,
//			filter->z,
//			filter->zdot,
//			filter->P11,
//			filter->P22,
//			filter->P33,
//			filter->P34,
//			filter->P44);
//	UARTwrite(buffer, len);
//	return;
//}

// Configure the UART and its pins.  This must be called before UARTprintf().
void ConfigureUART(void)
{
	// Enable the GPIO Peripheral used by the UART.
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	// Enable UART0
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

	// Configure GPIO Pins for UART mode.
	ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
	ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
	ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// Use the internal 16MHz oscillator as the UART clock source.
	ROM_UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

	// Edited by Zhengjie
	ROM_UARTEnable(UART0_BASE);

	// Initialize the UART for console I/O.
	UARTStdioConfig(0, 115200, 16000000);
}


/*
 * I2C_hw.cpp
 *
 *  Created on: Jun 24, 2015
 *      Author: Zhengjie
 */

#include "I2C_hw.hpp"

uint8_t Command[] = {LIDAR_REG_MEA, LIDAR_MEA_VALE, LIDAR_FULL_BYTE, PX4_MEA};
bool g_bI2CMDone = true;
tI2CMInstance g_sI2CMInst;
uint32_t LidarTime = 0, PX4Time = 0;

//******************************************************************************
// The interrupt handler for the I2C module.
void I2CMasterIntHandler(void)
{
	// Call the I2C master driver interrupt handler.
	I2CMIntHandler(&g_sI2CMInst);
}

// The function that is provided by this Run as a callback when I2C transactions have completed.
void I2CMCallback(void *pvData, uint_fast8_t ui8Status)
{
	// See if an error occurred.
	if(ui8Status != I2CM_STATUS_SUCCESS)
	{
		// An error occurred, so handle it here if required.
		UARTprintf("Error\n");
	}

	// Indicate that the I2C transaction has completed.
	g_bI2CMDone = true;
}

void ConfigUART(void)
{
	// Enable the GPIO Peripheral used by the UART.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	// Enable UART0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

	// Configure GPIO Pins for UART mode.
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// Use the internal 16MHz oscillator as the UART clock source.
	UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

	// Initialize the UART for console I/O.
	UARTStdioConfig(0, 115200, 16000000);
}

void ConfigI2C(void)
{
    // The I2C3 peripheral must be enabled before use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    // Configure the pin muxing for I2C3 functions on port D0 and D1.
    GPIOPinConfigure(GPIO_PD0_I2C3SCL);
    GPIOPinConfigure(GPIO_PD1_I2C3SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
    GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);

    // Initializes operation of the I2C Master block by configuring the bus speed(true for 400Kps)
    I2CMasterInitExpClk(I2C3_BASE, SysCtlClockGet(), true);

    I2CIntRegister(I2C3_BASE, I2CMasterIntHandler);

    // Enable interrupts to the processor.
    IntMasterEnable();
}


// Timer**********************************************************************************
void TimerInit(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC_UP);
	TimerClockSourceSet(TIMER0_BASE, TIMER_CLOCK_SYSTEM);
    TimerControlStall(TIMER0_BASE, TIMER_A, true);

	//System clock 50MHz = 0.020us = 20ns;
	//Setting Timer clock = 10us
	//Timer load value = 10/.020  =  500 clock ticks
    // 2^32-1 = 4294967295
	TimerLoadSet(TIMER0_BASE, TIMER_A, 4294967295);
	TimerEnable(TIMER0_BASE, TIMER_A);
}






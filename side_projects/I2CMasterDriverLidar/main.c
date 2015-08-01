// June 2015, Zhengjie Cui

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
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
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/i2c.h"
#include "utils/uartstdio.h"
#include "sensorlib/i2cm_drv.h"

// Define Lidar I2C Address.
#define LIDAR_I2C_ADDRESS      	0x62
#define LIDAR_REG_MEA     		0x00          // Register to write to initiate ranging.
#define LIDAR_MEA_VALE	        0x04          // Value to initiate ranging.
#define LIDAR_VEL				0x80		  // Change to Vel mode
#define LIDAR_FULL_BYTE		    0x8F          // Register to get both High and Low bytes in 1 call.
#define LIDAR_VEL_REG			0x09
#define LIDAR_VEL_SCALE_CMD		0x68
#define LIDAR_VEL_SCALE			0x14

// The I2C master driver instance data.
tI2CMInstance g_sI2CMLidarInst;

// A boolean that is set when an I2C transaction is completed.
volatile bool g_bI2CMLidarDone = false;

// The interrupt handler for the I2C module.
void I2CMLidarIntHandler(void)
{
	// Call the I2C master driver interrupt handler.
	I2CMIntHandler(&g_sI2CMLidarInst);
}

// The function that is provided by this Run as a callback when I2C transactions have completed.
void I2CMLidarCallback(void *pvData, uint_fast8_t ui8Status)
{
	// See if an error occurred.
	if(ui8Status != I2CM_STATUS_SUCCESS)
	{
		// An error occurred, so handle it here if required.
	}
	
	// Indicate that the I2C transaction has completed.
	g_bI2CMLidarDone = true;
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

    I2CIntRegister(I2C3_BASE, I2CMLidarIntHandler);

    // Initializes operation of the I2C Master block by configuring the bus speed(true for 400Kps)
    I2CMasterInitExpClk(I2C3_BASE, SysCtlClockGet(), true);

    // Enable interrupts to the processor.
    IntMasterEnable();

}

// The Lidar I2C master driver Run.
void I2CMLidarRun(void)
{
	uint8_t pui8Data[2];
	uint8_t pui8Command[] = {LIDAR_REG_MEA, LIDAR_MEA_VALE, LIDAR_VEL, LIDAR_FULL_BYTE, LIDAR_VEL_REG, LIDAR_VEL_SCALE_CMD, LIDAR_VEL_SCALE};

	// Write two bytes of data to the I2C device at address.
	g_bI2CMLidarDone = false;
	I2CMWrite(&g_sI2CMLidarInst, LIDAR_I2C_ADDRESS, pui8Command, 2, I2CMLidarCallback, 0);
	while(!g_bI2CMLidarDone)
	{
	}

    // Delay to get all 2 Byte data writen. About 15 milliseconds.
    SysCtlDelay(SysCtlClockGet() / (1000 * 3) * 15);

	// Read four bytes of data from the I2C device at address.
	g_bI2CMLidarDone = false;
	I2CMRead(&g_sI2CMLidarInst, LIDAR_I2C_ADDRESS, pui8Command + 3, 1, pui8Data, 2, I2CMLidarCallback, 0);
	while(!g_bI2CMLidarDone)
	{
	}

	uint32_t uDistance = (pui8Data[0] << 8) | pui8Data[1];

	UARTprintf("Distance %u\t\t\n", uDistance);

//	// Setting Scales for velociy
//	g_bI2CMLidarDone = false;
//	I2CMWrite(&g_sI2CMLidarInst, LIDAR_I2C_ADDRESS, pui8Command + 5, 2, I2CMLidarCallback, 0);
//	while(!g_bI2CMLidarDone)
//	{
//	}
//
//    // Delay to get all 2 Byte data writen. About 15 milliseconds.
//    SysCtlDelay(SysCtlClockGet() / (1000 * 3) * 10 );
//
//	// Change to Vel mode
//	g_bI2CMLidarDone = false;
//	I2CMWrite(&g_sI2CMLidarInst, LIDAR_I2C_ADDRESS, pui8Command + 1, 2, I2CMLidarCallback, 0);
//	while(!g_bI2CMLidarDone)
//	{
//	}
//
//    // Delay to get all 2 Byte data writen. About 15 milliseconds.
//    SysCtlDelay(SysCtlClockGet() / (1000 * 3) * 10 );
//
//	int8_t Vel[1];
//    // Read Vel
//	g_bI2CMLidarDone = false;
//	I2CMRead(&g_sI2CMLidarInst, LIDAR_I2C_ADDRESS, pui8Command + 4, 1, Vel, 1, I2CMLidarCallback, 0);
//	while(!g_bI2CMLidarDone)
//	{
//	}
//
//	int uVel = (int)Vel[0];
//    // Print Distance.
//    UARTprintf("Vel %d\t\t \n", uVel);
//
//    // Delay to get all 2 Byte data writen. About 15 milliseconds.
//    SysCtlDelay(SysCtlClockGet() / (1000 * 3) * 10 );
//
//    // Change back to the dis mode
//    uint8_t cmd[] = {LIDAR_REG_MEA, LIDAR_REG_MEA};
//	g_bI2CMLidarDone = false;
//	I2CMWrite(&g_sI2CMLidarInst, LIDAR_I2C_ADDRESS, cmd, 2, I2CMLidarCallback, 0);
//	while(!g_bI2CMLidarDone)
//	{
//	}
//
//    // Delay to get all 2 Byte data writen. About 15 milliseconds.
//    SysCtlDelay(SysCtlClockGet() / (1000 * 3) * 10 );
}

int main(void)
{
    // Setup the system clock to run at 40 MHz from PLL with crystal reference
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

	ConfigUART();

    // Print the welcome message to the terminal.
    UARTprintf("I2CMaster Driver Testing\n\r");

	ConfigI2C();

	// Initialize the I2C master driver. It is assumed that the I2C module has
	// already been enabled and the I2C pins have been configured.
	I2CMInit(&g_sI2CMLidarInst, I2C3_BASE, INT_I2C3, 0xff, 0xff, SysCtlClockGet());

	while (1)
	{
		I2CMLidarRun();
	}
}

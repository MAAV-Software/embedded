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
#include "driverlib/uart.h"
#include "driverlib/i2c.h"
#include "utils/uartstdio.h"
#include "sensorlib/i2cm_drv.h"

#define PX4_I2C_ADDRESS		0x48
#define PX4_MEA     		0x00

typedef struct _PX4_data
{
	uint16_t frame_count;// counts created I2C frames [#frames]
	int16_t pixel_flow_x_sum;// latest x flow measurement in pixels*10 [pixels]
	int16_t pixel_flow_y_sum;// latest y flow measurement in pixels*10 [pixels]
	int16_t flow_comp_m_x;// x velocity*1000 [millimeters/sec]
	int16_t flow_comp_m_y;// y velocity*1000 [millimeters/sec]
	int16_t qual;// Optical flow quality / confidence [0: bad, 255: maximum quality]
	int16_t gyro_x_rate; // latest gyro x rate [rad/sec]
	int16_t gyro_y_rate; // latest gyro y rate [rad/sec]
	int16_t gyro_z_rate; // latest gyro z rate [rad/sec]
	uint8_t gyro_range; // gyro range [0 .. 7] equals [50 deg/sec .. 2000 deg/sec]
	uint8_t sonar_timestamp;// time since last sonar update [milliseconds]
	int16_t ground_distance;// Ground distance in meters*1000 [millimeters]. Positive value: distance known. Negative value: Unknown distance
} PX4_data;

typedef union _PX4_frame
{
	char raw_8[sizeof(PX4_data)];
	PX4_data data;
} PX4_frame;

volatile PX4_frame px4_buffer;

int16_t px4_i2c_getHeight(void);
uint8_t px4_i2c_getTimestep(void);
uint16_t px4_i2c_get_frame_count();
int16_t  px4_i2c_get_pixel_flow_x_sum();
int16_t  px4_i2c_get_pixel_flow_y_sum();
int16_t  px4_i2c_get_flow_comp_m_x();
int16_t  px4_i2c_get_flow_comp_m_y();
int16_t  px4_i2c_get_qual();
int16_t  px4_i2c_get_gyro_x_rate();
int16_t  px4_i2c_get_gyro_y_rate();
int16_t  px4_i2c_get_gyro_z_rate();
uint8_t  px4_i2c_get_gyro_range();

// The I2C master driver instance data.
tI2CMInstance g_sI2CMPX4Inst;

// A boolean that is set when an I2C transaction is completed.
volatile bool g_bI2CMPX4Done = true;

// The interrupt handler for the I2C module.
void I2CMPX4IntHandler(void)
{
	// Call the I2C master driver interrupt handler.
	I2CMIntHandler(&g_sI2CMPX4Inst);
}

// The function that is provided by this Run as a callback when I2C transactions have completed.
void I2CMPX4Callback(void *pvData, uint_fast8_t ui8Status)
{
	// See if an error occurred.
	if(ui8Status != I2CM_STATUS_SUCCESS)
	{
		// An error occurred, so handle it here if required.
		UARTprintf("Error\n");
	}

	// Indicate that the I2C transaction has completed.
	g_bI2CMPX4Done = true;

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

    I2CIntRegister(I2C3_BASE, I2CMPX4IntHandler);

    // Enable interrupts to the processor.
    IntMasterEnable();

}

// The PX4 I2C master driver Run.
void I2CMPX4Run(void)
{
//	uint8_t pui8Data[22];
	uint8_t pui8Command[1] = {PX4_MEA};

	// Read four bytes of data from the I2C device at address.
	g_bI2CMPX4Done = false;
	I2CMRead(&g_sI2CMPX4Inst, PX4_I2C_ADDRESS, pui8Command, 1, px4_buffer.raw_8, 22, I2CMPX4Callback, 0);
	while(!g_bI2CMPX4Done)
	{
	}

    UARTprintf("%d\t%d\t%d\n", px4_i2c_getHeight(), px4_i2c_get_flow_comp_m_x(), px4_i2c_get_flow_comp_m_y());

//    snprintf(pcBuffer, 100, "y:%u\n", px4_i2c_get_flow_comp_m_y());
//
//    // Print flow y com.
//    UARTprintf(pcBuffer);


	// Delay to get all 2 Byte data writen. About 5 milliseconds.
	SysCtlDelay(SysCtlClockGet() / (1000 * 3) * 2 );

}

//**************************************MAIN FUNCTION**************************************************
int main(void)
{
    // Setup the system clock to run at 40 MHz from PLL with crystal reference
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

	ConfigUART();

    // Print the welcome message to the terminal.
    UARTprintf("I2CMaster Driver PX4 Testing\n\r");

	ConfigI2C();

	// Initialize the I2C master driver. It is assumed that the I2C module has
	// already been enabled and the I2C pins have been configured.
	I2CMInit(&g_sI2CMPX4Inst, I2C3_BASE, INT_I2C3, 0xff, 0xff, SysCtlClockGet());

	while (1)
	{
		I2CMPX4Run();
	}
}
//*****************************************************************************************************

uint16_t px4_i2c_get_frame_count()
{
	return px4_buffer.data.frame_count;
}

int16_t px4_i2c_get_pixel_flow_x_sum()
{
	return px4_buffer.data.pixel_flow_x_sum;
}

int16_t px4_i2c_get_pixel_flow_y_sum()
{
	return px4_buffer.data.pixel_flow_y_sum;
}

int16_t px4_i2c_get_flow_comp_m_x()
{
	return px4_buffer.data.flow_comp_m_x;
}

int16_t px4_i2c_get_flow_comp_m_y()
{
	return px4_buffer.data.flow_comp_m_y;
}

int16_t px4_i2c_get_qual()
{
	return px4_buffer.data.qual;
}

int16_t px4_i2c_get_gyro_x_rate()
{
	return px4_buffer.data.gyro_x_rate;
}

int16_t px4_i2c_get_gyro_y_rate()
{
	return px4_buffer.data.gyro_y_rate;
}

int16_t px4_i2c_get_gyro_z_rate()
{
	return px4_buffer.data.gyro_z_rate;
}

uint8_t px4_i2c_get_gyro_range()
{
	return px4_buffer.data.gyro_range;
}

uint8_t px4_i2c_getTimestep(void)
{
	return px4_buffer.data.sonar_timestamp;
}

int16_t px4_i2c_getHeight(void)
{
	return px4_buffer.data.ground_distance;
}

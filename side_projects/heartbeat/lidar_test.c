#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

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
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"

#include "utils/uartstdio.h"

#define LIDAR_ADDR 0x62
#define FULL_READ 0x8f
// System Clock Frequency
#define SYSCLOCK 80000000

typedef struct _lidar
{
	uint16_t distance; // distance measured
} lidar_t;

typedef union _lidar_frame
{
	char raw_8[sizeof(lidar_t)];
	lidar_t data;
} lidar_frame;

enum lidar_state_t
{
	UNINITIALIZED,
	SENDING_MEASUREMENT_COMMAND,
	SENDING_DIST_REQ,
	WAITING_FOR_RESPONSE,
	GET_FIRST_BYTE,
	GET_SECOND_BYTE,
	GET_INTERMEDIATE_BYTES,
	GET_LAST_BYTE,
	DATA_READY
};

volatile lidar_frame lidar;
volatile lidar_frame lidar_buffer;
volatile enum lidar_state_t lidar_state;
volatile uint32_t idx_lidar;
volatile bool data_fresh;
volatile bool lidar_can_transmit;

void master_I2C3_Int_Handler(void);
void lidar_measurement_aquisition(void);
void initiate_lidar_transmit(void);
void make_data_stale(void);
uint32_t get_lidar_distance(void);
bool lidar_transmit_done(void);

int main(void)
{
	/*
	 *  Set system clock to 80Mhz.
	 *  Note that SysCtlClockGet() has a bug for this frequency.
	 *  Use the #defined constant "SYSCLOCK" from utility.h instead.
	 */
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
				   SYSCTL_XTAL_16MHZ);

	// Init the LEDs on the Launchpad for debugging and init them to off
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

	GPIOPinConfigure(GPIO_PD0_I2C3SCL);
	GPIOPinConfigure(GPIO_PD1_I2C3SDA);
	GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
	GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);

	I2CMasterInitExpClk(I2C3_BASE, SYSCLOCK, true);  // set SCK to 400 Kbps
	I2CIntRegister(I2C3_BASE, master_I2C3_Int_Handler);
	//I2CMasterSlaveAddrSet(I2C0_BASE, LIDAR_ADDR, false);
	data_fresh = false;
	lidar_can_transmit = true;

    // Enable the GPIO Peripheral used by the UART0 on PA.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    // Enable UART0.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    // Configure GPIO Pins for UART mode.
    GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTConfigSetExpClk(UART0_BASE, SYSCLOCK, 115200,
	    				    (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
	    					 UART_CONFIG_PAR_NONE));
	// Configure UART Clock.
	UARTClockSourceSet(UART0_BASE, UART_CLOCK_SYSTEM);
	UARTStdioConfig(0, 115200, SYSCLOCK);

	// Enable all interrupts
	IntMasterEnable();

	uint32_t count = 0;
	uint8_t led_on = 0;
	uint32_t distance;
	//uint32_t aquisition_time = 0;

	UARTprintf("Welcome to the Pulsed Light lidar test program!\n\r");
	for (;;)
	{
		if (count > 1000000)
		{
			count = 0;
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, led_on);
			led_on ^= 1;
			UARTprintf("Loop Count = %u\n\r", count);
		}
		++count;

		if (lidar_can_transmit == true)
		{
			lidar_measurement_aquisition();
			initiate_lidar_transmit();
			lidar_can_transmit = false;
			UARTprintf("Initiating Transmit\n\r");
		}

		if (lidar_transmit_done() == true)
		{
			distance = get_lidar_distance();
			make_data_stale();
			lidar_can_transmit = true;
			UARTprintf(buffer, 100, "Distance = %u\n", distance);
		}
	}

	//return 0;
}


void lidar_measurement_aquisition(void)
{
	lidar_state = SENDING_MEASUREMENT_COMMAND;
	I2CMasterSlaveAddrSet(I2C0_BASE, LIDAR_ADDR, false); // i2c write
	I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	I2CMasterDataPut(I2C3_BASE, 0x00); // write to control register 0 (0x0)
	I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
	I2CMasterDataPut(I2C3_BASE, 0x04); // write data of 0x4 for measurement aquisition
	I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
	return;
}

void initiate_lidar_transmit(void)
{
	lidar_state = SENDING_DIST_REQ;
	I2CMasterSlaveAddrSet(I2C0_BASE, LIDAR_ADDR, false); // i2c write
	I2CMasterDataPut(I2C3_BASE, 0x8F); // write 0x8F to get 2-byte data in upcomming read
	I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_SEND);
	return;
}

void master_I2C3_Int_Handler(void)
{
	I2CMasterIntClear(I2C3_BASE);
	uint32_t errVal = I2CMasterErr(I2C3_BASE);
	char recv_char = I2CMasterDataGet(I2C3_BASE);

	if (lidar_state == SENDING_DIST_REQ)
	{
		idx_lidar = 0;
		lidar_state = GET_FIRST_BYTE;
		I2CMasterSlaveAddrSet(I2C3_BASE, LIDAR_ADDR, true); // i2c read
		I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
	}
	else if (lidar_state == GET_FIRST_BYTE) // First byte just came in.
	{
		lidar_buffer.raw_8[idx_lidar++] = recv_char;	// Put first byte in the array
		lidar_state = GET_SECOND_BYTE;
		I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
	}
	else if (lidar_state == GET_SECOND_BYTE) // second byte came in
	{
		lidar_buffer.raw_8[idx_lidar] = recv_char;
		I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

		if (data_fresh == false)
		{
			memcpy((void*)&lidar, (void*)&lidar_buffer, sizeof(lidar_frame));
			data_fresh = true;
		}
	}

	return;
}

bool lidar_transmit_done(void)
{
	return data_fresh;
}

void make_data_stale(void)
{
	data_fresh = false;
	return;
}

uint32_t get_lidar_distance(void)
{
	return (uint32_t)(lidar.data.distance);
}

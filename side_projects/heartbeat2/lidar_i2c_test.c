
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


typedef struct _px4
{
	uint16_t frame_count;
	int16_t pflow_x;
	int16_t pflow_y;
	int16_t cflow_x;
	int16_t cflow_y;
	int16_t quality;
	int16_t gyro_x_rate;
	int16_t gyro_y_rate;
	int16_t gyro_z_rate;
	uint8_t gyro_range;
	uint8_t sonar_timestamp;
	int16_t ground_dist;
} px4_t;

typedef union _px4_frame
{
	char raw_8[sizeof(px4_t)];
	px4_t data;
} px4_frame;

px4_frame px4_buffer;

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

lidar_frame lidar;
lidar_frame lidar_buffer;
enum lidar_state_t lidar_state;
uint32_t idx_lidar;
uint8_t data_fresh;
uint8_t lidar_can_transmit;

void master_I2C3_Int_Handler(void);
void lidar_measurement_aquisition(void);
void initiate_lidar_transmit(void);
void make_data_stale(void);
uint32_t get_lidar_distance(void);
uint8_t lidar_transmit_done(void);

void i2c_nack_int_handler(void);

int main(void)
{
	//  Set system clock to 80Mhz.
	//  Note that SysCtlClockGet() has a bug for this frequency.
	//  Use the #defined constant "SYSCLOCK" from utility.h instead.
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
				   SYSCTL_XTAL_16MHZ);

	// Init the LEDs on the Launchpad for debugging and init them to off
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 2);

//*********************************************************************************************
	// config i2c ch 3
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
	SysCtlPeripheralReset(SYSCTL_PERIPH_I2C3);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	GPIOPinConfigure(GPIO_PD0_I2C3SCL);
	GPIOPinConfigure(GPIO_PD1_I2C3SDA);
	GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
	GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);

	I2CMasterInitExpClk(I2C3_BASE, SYSCLOCK, false);  // set SCK to 100 KHz
	//I2CIntRegister(I2C3_BASE, i2c_nack_int_handler);
	//I2CMasterIntEnableEx(I2C3_BASE, I2C_MASTER_INT_NACK); // config to int on NACKs
	I2CMasterDisable(I2C3_BASE);
	I2CMasterEnable(I2C3_BASE);

	data_fresh = 0;
	lidar_can_transmit = 1;

//	********************************************************************************************
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
//	IntMasterEnable();

	//uint32_t count = 0;
	//uint8_t led_on = 0;
	//uint32_t distance;
	//uint32_t aquisition_time = 0;
	uint32_t error = 0;

//	PX4 Test section begins************************************************************************

//	UARTprintf("Welcome to the PX4 test program!\n\r");
//	I2CMasterSlaveAddrSet(I2C3_BASE, 0x46, false); // i2c write
//	I2CMasterDataPut(I2C3_BASE, 0x00); // write to 0x0
//	I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_SEND);
//	while (I2CMasterBusy(I2C3_BASE));

//	I2CMasterBurstLengthSet(I2C3_BASE, 22);
//	I2CMasterSlaveAddrSet(I2C3_BASE, 0x46, true); // i2c write
//	uint8_t idx = 0;
//
//	for (idx = 0; idx < 22; ++idx)
//	{
//		if (idx ==  0) I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
//		if (idx == 21) I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
//		else		   I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
//
//		while(I2CMasterBusy(I2C3_BASE));
//		error = I2CMasterErr(I2C3_BASE);
//		if (error != I2C_MASTER_ERR_NONE) UARTprintf("Error: %x\n\r", error);
//		px4_buffer.raw_8[idx] = I2CMasterDataGet(I2C3_BASE);
//	}
//
//	UARTprintf("\n\rSonar Dist: %d\n\r", px4_buffer.data.ground_dist);
//	PX4 Test Section Ends*******************************************************************************


//	Lidar Test Begins***********************************************************************************
	UARTprintf("Welcome to the Pulsed Light lidar test program!\n\r");
	SysCtlDelay(SYSCLOCK / 3);

	int count = 0;
	while (1)
	{
		count ++;
		I2CMasterSlaveAddrSet(I2C3_BASE, LIDAR_ADDR, false); // i2c write
		I2CMasterBurstLengthSet(I2C3_BASE, 2);
		I2CMasterDataPut(I2C3_BASE, 0x00);
		I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_START);
		while(I2CMasterBusy(I2C3_BASE));
		error = I2CMasterErr(I2C3_BASE);
		if (error != I2C_MASTER_ERR_NONE) UARTprintf("Error: %x\n\r", error);

		I2CMasterDataPut(I2C3_BASE, 0x04);
		I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
		while(I2CMasterBusy(I2C3_BASE));
		error = I2CMasterErr(I2C3_BASE);
		if (error != I2C_MASTER_ERR_NONE) UARTprintf("Error: %x\n\r", error);

//		UARTprintf("Sent 0x62, 0x00, 0x04\n\r");
		SysCtlDelay(SYSCLOCK / 3000 * 15); // delay 1*15 ms

//******************************************************************************************************
		I2CMasterDataPut(I2C3_BASE, FULL_READ); // write 0x8F to get 2-byte data in upcomming read
		I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_SEND);
		while(I2CMasterBusy(I2C3_BASE));
		error = I2CMasterErr(I2C3_BASE);
		if (error != I2C_MASTER_ERR_NONE) UARTprintf("Error: %x\n\r", error);
//		UARTprintf("Sent 0x62, 0x8F to set up the read\n\r");

		I2CMasterSlaveAddrSet(I2C3_BASE, LIDAR_ADDR, true); // i2c read
		I2CMasterBurstLengthSet(I2C3_BASE, 2);
		I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
		while(I2CMasterBusy(I2C3_BASE));
		error = I2CMasterErr(I2C3_BASE);
		if (error != I2C_MASTER_ERR_NONE) UARTprintf("Error: %x\n\r", error);
		uint32_t recv = (I2CMasterDataGet(I2C3_BASE)) << 8;

		I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
		while(I2CMasterBusy(I2C3_BASE));
		error = I2CMasterErr(I2C3_BASE);
		if (error != I2C_MASTER_ERR_NONE) UARTprintf("Error: %x\n\r", error);
		recv |= I2CMasterDataGet(I2C3_BASE);

		UARTprintf("Count: %d  Distance: %u\n\r", count, recv);

//		SysCtlDelay(SYSCLOCK / 3000);

	}
//	I2CMasterSlaveAddrSet(I2C3_BASE, LIDAR_ADDR, false); // i2c write
//	I2CMasterDataPut(I2C3_BASE, 0x00); // write to control register 0 (0x0)
//	I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_START);
////	while(I2CMasterErr(I2C3_BASE) != I2C_MASTER_ERR_NONE);
////	while(I2CMasterBusy(I2C3_BASE));
////	error = I2CMasterErr(I2C3_BASE);
////	if (error != I2C_MASTER_ERR_NONE) UARTprintf("Error: %x\n\r", error);
//	I2CMasterDataPut(I2C3_BASE, 0x04); // write data of 0x4 for measurement aquisition
//	I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
////	while(I2CMasterBusy(I2C3_BASE)); // Wait until the slave has received and acknowledged the data.
////	error = I2CMasterErr(I2C3_BASE);
////	if (error != I2C_MASTER_ERR_NONE) UARTprintf("Error: %x\n\r", error);
//	UARTprintf("Sent 0xC4, 0x00, 0x04\n\r");


//	I2CMasterDataPut(I2C3_BASE, 0x8F); // write 0x8F to get 2-byte data in upcomming read
//	I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_SEND);
//	while(I2CMasterBusy(I2C3_BASE));
//	error = I2CMasterErr(I2C3_BASE);
//	if (error != I2C_MASTER_ERR_NONE) UARTprintf("Error: %x\n\r", error);
//	UARTprintf("Sent 0xC4, 0x8F to set up the read\n\r");
//	UARTprintf("Initiating 2-byte read (0xC5)\n\r");

/*
	I2CMasterSlaveAddrSet(I2C3_BASE, LIDAR_ADDR, true); // i2c read
	I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
	while(I2CMasterBusy(I2C3_BASE));
	error = I2CMasterErr(I2C3_BASE);
	if (error != I2C_MASTER_ERR_NONE) UARTprintf("Error: %x\n\r", error);
	uint32_t recv = (I2CMasterDataGet(I2C3_BASE)) << 8;

	I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
	while(I2CMasterBusy(I2C3_BASE));
	error = I2CMasterErr(I2C3_BASE);
	if (error != I2C_MASTER_ERR_NONE) UARTprintf("Error: %x\n\r", error);
	recv |= I2CMasterDataGet(I2C3_BASE);

	UARTprintf("\n\r%u\n\r", recv);
	while (1)
	{

	}
	*/

	/*
	for (;;)
	{
		if (count > 10000)
		{
			count = 0;
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, led_on);
			led_on ^= 1;
			UARTprintf("Loop Count = %u\n\r", count);
		}
		++count;

		if (lidar_can_transmit)
		{
			lidar_measurement_aquisition();
			initiate_lidar_transmit();
			lidar_can_transmit = 0;
			UARTprintf("Initiating Transmit\n\r");
		}

		uint8_t isDone = lidar_transmit_done();
		if (isDone)
		{
			distance = get_lidar_distance();
			make_data_stale();
			lidar_can_transmit = 1;
			UARTprintf("Distance = %u\n", distance);
		}
	} */

	//return 0;
}


//void i2c_nack_int_handler(void)
//{
//	I2CMasterIntClearEx(I2C3_BASE, I2C_MASTER_INT_NACK);
//
//	UARTprintf("\n\rNack Interrupt\n\r");
//	uint32_t error = I2CMasterErr(I2C3_BASE);
//	UARTprintf("error = %x", error);
//	//while ((error == I2C_MASTER_ERR_AWQDDR_ACK) || (error == I2C_MASTER_ERR_DATA_ACK))
//	while (error != I2C_MASTER_ERR_NONE)
//	{
//		error = I2CMasterErr(I2C3_BASE);
//		UARTprintf("Waiting for ACK\n\r");
//	}
//	UARTprintf("ACK recieved\n\r");
//	return;
//}


/*

void lidar_measurement_aquisition(void)
{
	lidar_state = SENDING_MEASUREMENT_COMMAND;
	I2CMasterSlaveAddrSet(I2C3_BASE, LIDAR_ADDR, false); // i2c write
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
	I2CMasterSlaveAddrSet(I2C3_BASE, LIDAR_ADDR, false); // i2c write
	I2CMasterDataPut(I2C3_BASE, 0x8F); // write 0x8F to get 2-byte data in upcomming read
	I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_SEND);
	return;
}

void master_I2C3_Int_Handler(void)
{
	I2CMasterIntClear(I2C3_BASE);
	uint32_t err_val = I2CMasterErr(I2C3_BASE);
	char recv_char = I2CMasterDataGet(I2C3_BASE);

	UARTprintf("Iterrupt\tChar %u \tError Code %u\n\r", (uint8_t)recv_char, err_val);

	if (lidar_state == SENDING_DIST_REQ)
	{
		idx_lidar = 0;
		lidar_state = GET_FIRST_BYTE;
		I2CMasterSlaveAddrSet(I2C3_BASE, LIDAR_ADDR, true); // i2c read
		I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
		UARTprintf("Start Read\n\r");
	}
	else if (lidar_state == GET_FIRST_BYTE) // First byte just came in.
	{
		lidar_buffer.raw_8[idx_lidar++] = recv_char;	// Put first byte in the array
		lidar_state = GET_SECOND_BYTE;
		I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
		UARTprintf("Got first byte\n\r");
	}
	else if (lidar_state == GET_SECOND_BYTE) // second byte came in
	{
		lidar_buffer.raw_8[idx_lidar] = recv_char;
		I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
		UARTprintf("Got second byte\n\r");

		if (data_fresh == 0)
		{
			memcpy((void*)&lidar, (void*)&lidar_buffer, sizeof(lidar_frame));
			data_fresh = 1;
			UARTprintf("Copying data\n\r");
		}
	}

	return;
}

uint8_t lidar_transmit_done(void)
{
	return data_fresh;
}

void make_data_stale(void)
{
	data_fresh = 0;
	return;
}

uint32_t get_lidar_distance(void)
{
	return (uint32_t)(lidar.data.distance);
}
*/

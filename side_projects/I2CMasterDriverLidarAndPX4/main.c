// Created by Zhengjie, June, 20, 2015
// PX4 and Lidar both connected

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
#include "driverlib/timer.h"
#include "utils/uartstdio.h"
#include "sensorlib/i2cm_drv.h"

//***************************Command, Register and Address**********************
#define PX4_I2C_ADDRESS			0x46		// Define PX4 Address
#define PX4_MEA     			0x00		// Measurement Command
#define LIDAR_I2C_ADDRESS      	0x62		// Define Lidar I2C Address.
#define LIDAR_REG_MEA     		0x00        // Register to write to initiate ranging.
#define LIDAR_MEA_VALE	        0x04        // Value to initiate ranging.
#define LIDAR_FULL_BYTE		    0x8F        // Register to get both High and Low bytes in 1 call.

//***************************State Machine**************************************
// Keep track of state machine, meaning the state just done
typedef enum _state {Lidar1, Lidar2, PX4 } State;
State nextstate, currentstate;
uint32_t LidarTime = 0, PX4Time = 0;

//******************************PX4 Data Struct*********************************
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
volatile uint8_t lidar_buffer[2];
volatile uint8_t Command[] = {LIDAR_REG_MEA, LIDAR_MEA_VALE, LIDAR_FULL_BYTE, PX4_MEA};

uint32_t lidar_i2c_getDistance(void);
int16_t px4_i2c_getHeight(void);
uint8_t px4_i2c_getTimestep(void);
uint16_t px4_i2c_get_frame_count(void);
int16_t  px4_i2c_get_pixel_flow_x_sum(void);
int16_t  px4_i2c_get_pixel_flow_y_sum(void);
int16_t  px4_i2c_get_flow_comp_m_x(void);
int16_t  px4_i2c_get_flow_comp_m_y(void);
int16_t  px4_i2c_get_qual(void);
int16_t  px4_i2c_get_gyro_x_rate(void);
int16_t  px4_i2c_get_gyro_y_rate(void);
int16_t  px4_i2c_get_gyro_z_rate(void);
uint8_t  px4_i2c_get_gyro_range(void);

// The I2C master driver instance data.
tI2CMInstance g_sI2CMInst;

// A boolean that is set when an I2C transaction is completed.
volatile bool g_bI2CMDone = true;

//volatile bool g_LidarStart = true;
//volatile bool g_LidarDone = true;
//volatile bool g_PX4Done = true;

// The interrupt handler for the I2C module.
void I2CMasterIntHandler(void);

// The function that is provided by this Run as a callback when I2C transactions have completed.
void I2CMCallback(void *, uint_fast8_t);

void ConfigUART(void);

void ConfigI2C(void);

void TimerInit(void);

// The PX4 I2C master driver Run.
void I2CMPX4Run(void);


int main(void)
{
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
	currentstate = Lidar1;

	while(1)
	{
		uint32_t gettime = TimerValueGet(TIMER0_BASE, TIMER_A);

		if (g_bI2CMDone)
		{
			switch(currentstate){
			case Lidar1:
				if (gettime - PX4Time > 50000)// wait for 1ms after px4 done
				{
					PX4Time = gettime; // the time of px4 start
					g_bI2CMDone = false;
					I2CMRead(&g_sI2CMInst, PX4_I2C_ADDRESS, Command + 3, 1, px4_buffer.raw_8, 22, I2CMCallback, 0);
					while(!g_bI2CMDone){};
					UARTprintf("Ground Dis(mm) PX4:%u\t", px4_i2c_getHeight());
					nextstate = PX4;
				}
				break;
			case PX4:
				if (gettime - LidarTime > 750000) // wait for 15ms after lidar1 done
				{
					LidarTime = gettime; // the time of lidar2 start
					g_bI2CMDone = false;
					I2CMRead(&g_sI2CMInst, LIDAR_I2C_ADDRESS, Command + 2, 1, lidar_buffer, 2, I2CMCallback, 0);
					while(!g_bI2CMDone){};
					nextstate = Lidar2;
					UARTprintf("Ground Dis(cm) Lidar:%u\n", lidar_i2c_getDistance());
				}
				break;
			case Lidar2:
				if (gettime - LidarTime > 5000) //wait for 0.1ms after lidar2 done
				{
					LidarTime = gettime; // the time of lidar start
					g_bI2CMDone = false;
					I2CMWrite(&g_sI2CMInst, LIDAR_I2C_ADDRESS, Command, 2, I2CMCallback, 0);
					while(!g_bI2CMDone){};
					nextstate = Lidar1;
				}
				break;
			}
			currentstate = nextstate;
		}

	}
}

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


//******************************************************************************
uint32_t lidar_i2c_getDistance(void)
{
	return (lidar_buffer[0] << 8) | lidar_buffer[1];
}
uint16_t px4_i2c_get_frame_count(void)
{
	return px4_buffer.data.frame_count;
}

int16_t px4_i2c_get_pixel_flow_x_sum(void)
{
	return px4_buffer.data.pixel_flow_x_sum;
}

int16_t px4_i2c_get_pixel_flow_y_sum(void)
{
	return px4_buffer.data.pixel_flow_y_sum;
}

int16_t px4_i2c_get_flow_comp_m_x(void)
{
	return px4_buffer.data.flow_comp_m_x;
}

int16_t px4_i2c_get_flow_comp_m_y(void)
{
	return px4_buffer.data.flow_comp_m_y;
}

int16_t px4_i2c_get_qual(void)
{
	return px4_buffer.data.qual;
}

int16_t px4_i2c_get_gyro_x_rate(void)
{
	return px4_buffer.data.gyro_x_rate;
}

int16_t px4_i2c_get_gyro_y_rate(void)
{
	return px4_buffer.data.gyro_y_rate;
}

int16_t px4_i2c_get_gyro_z_rate(void)
{
	return px4_buffer.data.gyro_z_rate;
}

uint8_t px4_i2c_get_gyro_range(void)
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
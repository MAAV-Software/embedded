/*
 * imu_uart.c
 *
 *  Created on: Feb 21, 2015
 *      Authors: Carl Chan Zhengjie Cui
 */


#include "imu_uart.h"

volatile unsigned char g_IMU_Data_Received[79];
volatile const unsigned char g_IMU_Address_Sent[] = {0xCC};
volatile uint8_t g_IMU_Index = 0;
volatile uint8_t g_IMU_DataLength = 79;
volatile uint32_t g_one_sec = 0;
volatile uint8_t g_IMU_Status = 0;
IMU_data new_data;

void imu_uart_config_sys_clock(void)
{
    // Set the clocking to run from the PLL at 50MHz
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
//    SysCtlPeripheralClockGating(true);
}

//****************************************************************************************
void imu_uart_config_LED(void)
{
	// Enable the GPIO port that is used for the on-board LED.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

	// Enable the GPIO pins for the LED (PF1 & PF2 & PF3).
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1);

	// Turn off All LEDs.
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1, 0);
}

//****************************************************************************************
void imu_uart_toggle_LED(uint32_t led, uint32_t time)
{
	// Turn off LED.
	GPIOPinWrite(GPIO_PORTF_BASE, led, led);
	SysCtlDelay(time);
	// Turn on LED.
	GPIOPinWrite(GPIO_PORTF_BASE, led, 0);
	SysCtlDelay(time);
}

//****************************************************************************************
//void imu_uart_config_SW(void)
//{
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
//	GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);
//	GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA , GPIO_PIN_TYPE_STD_WPU);
//	GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_RISING_EDGE);
//
//	// Register and Enable GPIO Interrupt Handler
//	GPIOIntRegister(GPIO_PORTF_BASE, imu_uart_SW_int_handler);
//	IntEnable(INT_GPIOF);
//	GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_4);
//	// IntMasterEnable();
//}

//void imu_uart_SW_int_handler(void)
//{
//	uint32_t ui32Status = GPIOIntStatus(GPIO_PORTF_BASE, true);
//	GPIOIntClear(GPIO_PORTF_BASE, ui32Status);
//
//	if (ui32Status == GPIO_PIN_4)
//	{
//		g_IMU_Status = 1;
//
//		GPIOPinWrite(GPIO_PORTF_BASE, RED_LED | BLUE_LED | GREEN_LED, 0);
//		// Toggle GREEN LED, stop signal
//		imu_uart_toggle_LED(GREEN_LED, g_one_sec);
//	}
//
//}

//****************************************************************************************
void imu_uart_config_uart(void)
{
    // Enable the GPIO Peripheral used by the UART1 on PC. UART0 on PA.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Enable UART1 and UART0.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
//    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART1);
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
//    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART0);

    // Configure GPIO Pins for UART mode.
    GPIOPinConfigure(GPIO_PC4_U1RX);
    GPIOPinConfigure(GPIO_PC5_U1TX);
    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

//    GPIOPinConfigure(GPIO_PA0_U0RX);
//	GPIOPinConfigure(GPIO_PA1_U0TX);
//	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// Configure UART Clock.
	UARTClockSourceSet(UART1_BASE, UART_CLOCK_SYSTEM);
//	UARTClockSourceSet(UART0_BASE, UART_CLOCK_SYSTEM);

	//Configure UART for operation in the specified data format.
    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
//    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    // Register and Enable UART1 RX Interrupt.
    UARTIntRegister(UART1_BASE, imu_uart_int_handler);
    IntMasterEnable();
    IntEnable(INT_UART1);
    UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);

}

//****************************************************************************************
void imu_uart_int_handler(void)
{
    uint32_t ui32Status;

    // Get the interrrupt status.
    ui32Status = UARTIntStatus(UART1_BASE, true);

    // Clear the asserted interrupts.
    UARTIntClear(UART1_BASE, ui32Status);

    // Loop while there are characters in the receive FIFO.
    while(UARTCharsAvail(UART1_BASE))
    {
        // Read next data.
        unsigned char data_get = UARTCharGetNonBlocking(UART1_BASE);
        if (data_get == 0xCC)
        {
        	g_IMU_Index = 0;
        }

        g_IMU_Data_Received[g_IMU_Index++] = data_get;

		if (g_IMU_Index >= g_IMU_DataLength) imu_uart_parseData((uint8_t *)g_IMU_Data_Received);

    }

    // Toggle Green LED to indicate we are in the interrupt
//    imu_uart_toggle_LED(GREEN_LED, g_one_sec/1000);
}

//****************************************************************************************
void imu_uart_send(uint32_t Base, const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    // Loop while there are more characters to send.
    while(ui32Count--)
    {
        // Write the next character to the UART.
        UARTCharPut(Base, *pui8Buffer++);
    }
}

//****************************************************************************************
void imu_uart_parseData(unsigned char * data)
{
	// Check checksum
	short i, tChksum = 0;

	for (i = 0; i < 77; i++)
	{
		tChksum += data[i];
	}

	short tResponseChksum = 0;
	tResponseChksum = data[77] << 8;
	tResponseChksum += data[78];

	if (tChksum != tResponseChksum)
	{
		return;
	}

	new_data.AccX = Bytes2Float(data, 1);
	new_data.AccY = Bytes2Float(data, 5);
	new_data.AccZ = Bytes2Float(data, 9);
	new_data.AngRateX = Bytes2Float(data, 13);
	new_data.AngRateY = Bytes2Float(data, 17);
	new_data.AngRateZ = Bytes2Float(data, 21);
	new_data.MagX = Bytes2Float(data, 25);
	new_data.MagY = Bytes2Float(data, 29);
	new_data.MagZ = Bytes2Float(data, 33);
	new_data.M11 = Bytes2Float(data, 37);
	new_data.M12 = Bytes2Float(data, 41);
	new_data.M13 = Bytes2Float(data, 45);
	new_data.M21 = Bytes2Float(data, 49);
	new_data.M22 = Bytes2Float(data, 53);
	new_data.M23 = Bytes2Float(data, 57);
	new_data.M31 = Bytes2Float(data, 61);
	new_data.M32 = Bytes2Float(data, 65);
	new_data.M33 = Bytes2Float(data, 69);
	new_data.Timer = Bytes2Int(data, 73);

//	imu_uart_toggle_LED(RED_LED, g_one_sec/1000*(atan2(new_data.M23, new_data.M33)+3.15));
//	imu_uart_toggle_LED(RED_LED, g_one_sec/1000*(asin(-new_data.M13)+3.15));
//	imu_uart_toggle_LED(RED_LED, g_one_sec/1000*(atan2(new_data.M12, new_data.M11)+3.15));


	return;
}

uint32_t Bytes2Int(unsigned char *raw, unsigned int i)
{
	union B2I{
	   unsigned char buf[4];
	   uint32_t number;
	}data;

	data.buf[0] = raw[i+3];
	data.buf[1] = raw[i+2];
	data.buf[2] = raw[i+1];
	data.buf[3] = raw[i+0];

	return data.number;
}

float Bytes2Float(unsigned char *raw, unsigned int i)
{
	union B2F{
	   unsigned char buf[4];
	   float number;
	}data;

	data.buf[0] = raw[i+3];
	data.buf[1] = raw[i+2];
	data.buf[2] = raw[i+1];
	data.buf[3] = raw[i+0];

	return data.number;
}

float imu_uart_getAccX()
{
	return new_data.AccX;
}

float imu_uart_getAccY()
{
	return new_data.AccY;
}

float imu_uart_getAccZ()
{
	return new_data.AccZ;
}

float imu_uart_getRoll()
{
	return atan2(new_data.M23, new_data.M33);
}

float imu_uart_getPitch()
{
	return asin(-new_data.M13);
}

float imu_uart_getYaw()
{
	return atan2(new_data.M12, new_data.M11);
}

float imu_uart_getAngRateX()
{
	return new_data.AngRateX;
}

float imu_uart_getAngRateY()
{
	return new_data.AngRateY;
}

float imu_uart_getAngRateZ()
{
	return new_data.AngRateZ;
}

uint32_t imu_uart_getTimer()
{
	return new_data.Timer/62500;
}

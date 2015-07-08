#include "Imu_hw.hpp"

uint8_t imuRawIn[IMU_DATA_LENGTH];
uint8_t imuRawFinal[IMU_DATA_LENGTH];
bool imuDone = false;
uint8_t imuIndex = 0;
uint32_t imu_time = 0;
uint8_t imuCmd = (uint8_t)ALL_VAL_CMD;

void imu_uart_config_sys_clock(void)
{
    // Set the clocking to run from the PLL at 50MHz
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

}

//****************************************************************************************
// TODO Take in PIN, PORT, BASE, etc as arguments
void imu_uart_config_uart(void)
{
    // Enable the GPIO Peripheral used by the UART1 on PC. UART0 on PA.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Enable UART1 and UART0.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);

//    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Configure GPIO Pins for UART mode.
    GPIOPinConfigure(GPIO_PC4_U1RX);
    GPIOPinConfigure(GPIO_PC5_U1TX);
    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

//    GPIOPinConfigure(GPIO_PA0_U0RX);
//	GPIOPinConfigure(GPIO_PA1_U0TX);
//	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// Configure UART Clock.
	UARTClockSourceSet(UART1_BASE, UART_CLOCK_SYSTEM);

//	UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
	//TODO Replace SysCtlClockGet() has a bug for 80 MHz clock freq, so use our custome define here
	//Configure UART for operation in the specified data format.
    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
//    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    UARTEnable(UART1_BASE);
//    UARTEnable(UART0_BASE);

    // Initialize the UART0 for console I/O.
//	UARTStdioConfig(0, 115200, 16000000);

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
    while (UARTCharsAvail(UART1_BASE))
    {
        // Read next data.
        uint8_t data_get = UARTCharGetNonBlocking(UART1_BASE);

        if ((data_get == (uint8_t)ALL_VAL_CMD) &&
        		((imuIndex == 0) || (imuIndex >= (uint8_t)IMU_DATA_LENGTH)))
        {
        	imuIndex = 0;
        	imuDone = false;
        }

        imuRawIn[imuIndex++] = data_get;

		if (imuIndex >= (uint8_t)IMU_DATA_LENGTH)
		{
			//imu_uart_toggle_LED(BLUE_LED, SysCtlClockGet()/3/1000);
//			std::memcpy(imuRawFinal, imuRawIn, sizeof(uint8_t[IMU_DATA_LENGTH]));
			imu_memcpy(imuRawFinal, imuRawIn, IMU_DATA_LENGTH);

			imuDone = true;
		}

    }

}

//****************************************************************************************
void imu_uart_send(uint32_t Base, const uint8_t *pui8Buffer, uint32_t ui32Count)
{
	//uint32_t count = ui32Count;
    // Loop while there are more characters to send.
    while (ui32Count--)
    {
        // Write the next character to the UART.
        UARTCharPut(Base, *pui8Buffer++);
    }
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

// Timer**********************************************************************************
void imu_uart_timer_init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC_UP);
	TimerClockSourceSet(TIMER0_BASE, TIMER_CLOCK_SYSTEM);
    TimerControlStall(TIMER0_BASE, TIMER_A, true);

	//System clock 50MHz = 0.020us = 20ns;
	//Setting Timer clock = 10us
	//Timer load value = 10/.020  =  500 clock ticks
//	TimerLoadSet(TIMER0_BASE, TIMER_A, 4294967295);
	TimerEnable(TIMER0_BASE, TIMER_A);
//	ui32_current_time = 0;
}


void imu_memcpy(uint8_t* char_out, const uint8_t* char_in, int len)
{
//	int i = 0;
	for (int i = 0; i<len; i++) char_out[i] = char_in[i];
}

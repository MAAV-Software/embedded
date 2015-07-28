#include <stdint.h>
#include <cstdlib>
#include <cstring>

#include "DataLinkHw.hpp"

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
//#include "inc/hw_ints.h"
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

#include "time_util.h"
#include "LED.h"

using namespace std;

// globals that are externed
bool DLINK_RX_DONE = false;
uint8_t DLINK_RX_BUF[DLINK_UART_BUF_LEN];
uint8_t DLINK_RX_WORKING_BUF[DLINK_UART_BUF_LEN];
volatile uint8_t RX_IDX = 0;

// global that's not externed
uint32_t UART_BASE = UART0_BASE;

// declare ISR
static void DataLinkUartIntHandler();


// ISR definition
static void DataLinkUartIntHandler()
{
	// Get the interrrupt status.
	uint32_t status = MAP_UARTIntStatus(UART_BASE, true);

	// Clear the asserted interrupts.
	MAP_UARTIntClear(UART_BASE, status);

	// Loop while there are characters in the receive FIFO.
	while (MAP_UARTCharsAvail(UART_BASE))
	{
		DLINK_RX_WORKING_BUF[RX_IDX++] = MAP_UARTCharGetNonBlocking(UART_BASE);

		if (RX_IDX >= DLINK_UART_BUF_LEN)
		{
			RX_IDX = 0;
			memcpy(DLINK_RX_BUF, DLINK_RX_WORKING_BUF, DLINK_UART_BUF_LEN);
			DLINK_RX_DONE = true;
			break;
		}
		else
			DLINK_RX_DONE = false;
	}


}

void DataLinkUartConfig(const uint32_t sysctlPeriphUart,
						const uint32_t sysctlPeriphGPIO,
						const uint32_t rxPinConfig,
						const uint32_t txPinConfig,
						const uint32_t gpioPortBase,
						const uint32_t gpioRxPin,
						const uint32_t gpioTxPin,
						const uint32_t uartBase,
						const uint32_t uartInterrupt)
{
	UART_BASE = uartBase;

	// Enable the GPIO Peripheral used by the UART1 on PC. UART0 on PA.
    MAP_SysCtlPeripheralEnable(sysctlPeriphGPIO);
    MAP_SysCtlPeripheralEnable(sysctlPeriphUart);

    // Configure GPIO Pins for UART mode.
    MAP_GPIOPinConfigure(rxPinConfig);
    MAP_GPIOPinConfigure(txPinConfig);
    MAP_GPIOPinTypeUART(gpioPortBase, gpioRxPin | gpioTxPin);

	// Configure UART Clock.
	MAP_UARTClockSourceSet(uartBase, UART_CLOCK_SYSTEM);

	//Configure UART for operation in the specified data format.
    MAP_UARTConfigSetExpClk(uartBase, SYSCLOCK, 115200,
			(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    MAP_UARTEnable(uartBase);

    // Register and Enable UART1 RX Interrupt.
    UARTIntRegister(uartBase, DataLinkUartIntHandler);
    MAP_IntMasterEnable();
    MAP_IntEnable(uartInterrupt);
    MAP_UARTIntEnable(uartBase, UART_INT_RX | UART_INT_RT);

	// delay for initialization
	for (int i = 0; i < 3; ++i) Toggle_LED(BLUE_LED, SYSCLOCK / 3 / 2);//MAP_SysCtlDelay(SYSCLOCK / 3);

	RX_IDX = 0;
	DLINK_RX_DONE = false;
}

void DataLinkUartSend(const uint8_t *buf, uint32_t len)
{
	while (len-- > 0) MAP_UARTCharPut(UART_BASE, *buf++);
}

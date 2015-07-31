#include <stdint.h>
#include <cstdlib>
#include <cstring>

#include "DataLinkHw.hpp"
#include "messaging/RingBuffer.hpp"

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
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


// global that's not externed
volatile uint32_t UART_BASE = UART0_BASE;

// globals that are externed
RingBuffer<256> DL_RBUFF; // init rbuff with rbBack array

// ISR definition
void DataLinkUartIntHandler()
{
	// Get the interrrupt status.
	uint32_t status = MAP_UARTIntStatus(UART_BASE, true);

	// Clear the asserted interrupts.
	MAP_UARTIntClear(UART_BASE, status);

	// Loop while there are characters in the receive FIFO.
	while (MAP_UARTCharsAvail(UART_BASE))
		DL_RBUFF.push(MAP_UARTCharGetNonBlocking(UART_BASE));
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
	for (int i = 0; i < 3; ++i) Toggle_LED(RED_LED, SYSCLOCK / 3 / 2);
}

void DataLinkUartSend(const uint8_t *buf, uint32_t len)
{
	//while (len-- > 0) MAP_UARTCharPut(UART_BASE, *buf++);
	for (uint32_t i = 0; i < len; ++i) MAP_UARTCharPut(UART_BASE, buf[i]);
}

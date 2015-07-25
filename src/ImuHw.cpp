#include <stdint.h>

#include "ImuHw.hpp"

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

uint8_t imuRawIn[IMU_DATA_LENGTH];
uint8_t imuRawFinal[IMU_DATA_LENGTH];
bool imuDone = false;
uint8_t imuIndex = 0;
uint32_t imuTime = 0;
uint8_t imuCmd = 0xCC;

// local global, not externed
uint32_t uartBase = UART1_BASE;

//****************************************************************************************
void imuUartConfig(const uint32_t sysctlPeriphUart,
				   const uint32_t sysctlPeriphGPIO,
				   const uint32_t rxPinConfig,
				   const uint32_t txPinConfig,
				   const uint32_t gpioPortBase,
				   const uint32_t gpioRxPin,
				   const uint32_t gpioTxPin,
				   const uint32_t _uartBase,
				   const uint32_t uartInterrupt)
{
	uartBase = _uartBase;
    
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
    UARTIntRegister(uartBase, imuUartIntHandler);
    MAP_IntMasterEnable();
    MAP_IntEnable(uartInterrupt);
    MAP_UARTIntEnable(uartBase, UART_INT_RX | UART_INT_RT);
    
	// delay for initialization
	for (int i = 0; i < 3; ++i) Toggle_LED(BLUE_LED, SYSCLOCK / 3 / 2);//MAP_SysCtlDelay(SYSCLOCK / 3);
	
	imuDone = false;
}

//****************************************************************************************
void imuUartIntHandler()
{
    // Get the interrrupt status.
    uint32_t status = MAP_UARTIntStatus(uartBase, true);
	
    // Clear the asserted interrupts.
    MAP_UARTIntClear(uartBase, status);
	
    // Loop while there are characters in the receive FIFO.
    while (MAP_UARTCharsAvail(uartBase))
    {
        // Read next data.
        uint8_t dataGet = MAP_UARTCharGetNonBlocking(uartBase);

        if ((dataGet == imuCmd) &&
        		((imuIndex == 0) || (imuIndex >= (uint8_t)IMU_DATA_LENGTH)))
        {
        	imuIndex = 0;
        	imuDone = false;
        }

        imuRawIn[imuIndex++] = dataGet;

		if (imuIndex >= (uint8_t)IMU_DATA_LENGTH)
		{
			imuMemcpy(imuRawFinal, imuRawIn, (int)IMU_DATA_LENGTH);
			imuDone = true;
		}
    }
}

//****************************************************************************************
void imuUartSend(const uint8_t *buffer, const uint32_t count)
{
    // Loop while there are more characters to send
    for (int i = 0; i < count; ++i) MAP_UARTCharPut(uartBase, *buffer++);
}

// TODO May want to rename as MemCpy and move to MAAV utility library
void imuMemcpy(uint8_t* out, const uint8_t* in, const int len)
{
	for (int i = 0; i < len; i++) out[i] = in[i];
}

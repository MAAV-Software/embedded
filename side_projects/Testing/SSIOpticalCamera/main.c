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
#include "driverlib/ssi.h"

#include "utils/uartstdio.h"


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

//void SSI_Int_Handler(void)
//{
//	SSIIntClear(SSI0_BASE,ff);
//}

void ConfigSPI(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
	GPIOPinConfigure(GPIO_PA2_SSI0CLK);
	GPIOPinConfigure(GPIO_PA3_SSI0FSS);
	GPIOPinConfigure(GPIO_PA4_SSI0RX);
	GPIOPinConfigure(GPIO_PA5_SSI0TX);

	GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5);
	// 8bit package, 2000000 bit  = 2Mhz rate, running at 2Mz/8 = ? frequency
	SSIClockSourceSet(SSI0_BASE, SSI_CLOCK_SYSTEM);

	SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_3, SSI_MODE_MASTER, 2000000, 8);


	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);

	SSIEnable(SSI0_BASE);

//	SSIIntRegister(SSI0_BASE, SSI_Int_Handler);
//	IntMasterEnable();
//	IntEnable(INT_SSI0);
//	SSIIntEnable(SSI0_BASE, FF);

}

int main(void)
{
    // Setup the system clock to run at 80 MHz from PLL with crystal reference
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

	ConfigUART();

    // Print the welcome message to the terminal.
    UARTprintf("SPI Testing\n\r");

	ConfigSPI();
	uint32_t garbage;
	// Empty the FIFO buffer
	while(SSIDataGetNonBlocking(SSI0_BASE, &garbage));
//    while(SSIBusy(SSI0_BASE)){}


	uint32_t ID;
	uint32_t i = 0;
	while (1)
	{
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
		SSIDataPut(SSI0_BASE, 0x00);// Product ID
	    while(SSIBusy(SSI0_BASE)){}
	    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
		//wait for 50 usec
		SysCtlDelay(SysCtlClockGet()/3/1000/1000*50);

		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
		SSIDataPut(SSI0_BASE, 0x00);// Write dummy to activate CLK
		SSIDataGet(SSI0_BASE, &ID);
		SSIDataPut(SSI0_BASE, 0x00);// Write dummy to activate CLK
		SSIDataGet(SSI0_BASE, &ID);
		SSIDataPut(SSI0_BASE, 0x00);// Write dummy to activate CLK
		SSIDataGet(SSI0_BASE, &ID);
		SSIDataPut(SSI0_BASE, 0x00);// Write dummy to activate CLK
		SSIDataGet(SSI0_BASE, &ID);
		SSIDataPut(SSI0_BASE, 0x00);// Write dummy to activate CLK
		SSIDataGet(SSI0_BASE, &ID);
		SSIDataPut(SSI0_BASE, 0x00);// Write dummy to activate CLK
		SSIDataGet(SSI0_BASE, &ID);
		SSIDataPut(SSI0_BASE, 0x00);// Write dummy to activate CLK
		SSIDataGet(SSI0_BASE, &ID);
		while(SSIBusy(SSI0_BASE)){}

		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
//			DataIn[i] &=0x00FF;

		UARTprintf("count:%d\tID:%d\n\r",i,ID);
//		UARTprintf("Motion:%d\tDx:%d\tDy:%d\tSQUAL:%d\tShuttH:%d\tShuttL:%d\tPixel:%d\n\r",DataIn[0],DataIn[1],DataIn[2],DataIn[3],DataIn[4],DataIn[5],DataIn[6]);
		SysCtlDelay(SysCtlClockGet()/3/1000/10);

		while(SSIDataGetNonBlocking(SSI0_BASE, &garbage));

	}
}

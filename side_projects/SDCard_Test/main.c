#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
//#include "inc/tm4c123gh6pm.h"

#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"

#include "utils/uartstdio.h"

#include "sdlib/diskio.h"
#include "sdlib/ff.h"
#include "sdlib/integer.h"
#include "sdlib/ffconf.h"

static FATFS g_sFatFs;			// FatFs work area needed for each volume
static FIL g_sFileObject;		// File object needed for each open file
//static DIR g_sDirObject;
//static FILINFO g_sFileInfo;

void ConfigureUART(void){

	// Enable the peripherals used by UART
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

	// Set GPIO A0 and A1 as UART pins.
	ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
	ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
	ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// Configure UART clock using UART utils
	ROM_UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
	UARTStdioConfig(0, 115200, 16000000);
}

int main(void) {
	
	// Enable lazy stacking
	ROM_FPULazyStackingEnable();

	// Set the system clock to run at 40Mhz off PLL with external crystal as reference.
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

	ConfigureUART();

	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

	FRESULT iFResult;
	iFResult = f_mount(&g_sFatFs, "", 1);

	if(iFResult != FR_OK)  UARTprintf("f_mount error: %i\n", iFResult);
	else UARTprintf("\nMounted SD card\n");

	iFResult = f_open(&g_sFileObject, "data.txt", FA_WRITE | FA_CREATE_ALWAYS);
	if(iFResult != FR_OK) UARTprintf("Error creating file data.txt: %i\n", iFResult);
	else UARTprintf("\nCreated file data.txt\n");


	int a=0;
	unsigned int b;
	static char buffer[200] = "";
	while (a < 10)
	{
	    snprintf(buffer, 100, "time:%u\trow:%d\tpitch:%d\tyaw:%d\n", (uint32_t)(1000000), (int32_t)(1e7*1.52), (int32_t)(1e7*1.52), (int32_t)(1e7*1.52));
	    iFResult = f_write(&g_sFileObject, buffer, strlen(buffer), &b);
	    if(iFResult != FR_OK) UARTprintf("Error writing to data.txt: %i\n", iFResult);
	    else UARTprintf("Wrote %i words to data.txt: %i\n", b);
	    a++;
	}

	f_close(&g_sFileObject);

	// Unregister a work area before discard it
	f_mount(0, "", 1);

	while(1);
}

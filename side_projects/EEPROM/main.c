#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/flash.h"
#include "driverlib/eeprom.h"

#define NUM_FLOAT 24

union F2B
{
   unsigned char buf[4 * NUM_FLOAT];
   float number[NUM_FLOAT];
}datain;

union B2F
{
   unsigned char buf[4 * NUM_FLOAT];
   float number[NUM_FLOAT];
}dataout;

int main(void)
{
	uint32_t pui32Data[2];
	uint32_t pui32Read[2];
	pui32Data[0] = 0x12345678;
	pui32Data[1] = 0x56789abc;

	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x00);

	SysCtlDelay(20000000);

//	// Flash
//	FlashErase(0x10000);
//	FlashProgram(pui32Data, 0x10000, sizeof(pui32Data));
//	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x02);
//	SysCtlDelay(20000000);

	float floatArray[NUM_FLOAT] = {1.0, 2.1, 12.89,1.0, 2.1, 12.89,1.0, 2.1, 12.89,1.0, 2.1, 12.89,1.0, 2.1, 12.89,1.0, 2.1, 12.89,1.0, 2.1, 12.89,1.0, 2.1, 12.89};
	float readArray[NUM_FLOAT];

	int i = 0;
	for (i = 0; i < NUM_FLOAT; i++)
	{
		datain.number[i] = floatArray[i];
	}

	// EEPROM
	SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
	EEPROMInit();
	EEPROMMassErase();
//	EEPROMRead(pui32Read, 0x0, sizeof(pui32Read));

//	EEPROMProgram(datain.buf, 0x0, sizeof(datain.buf));
//
//	EEPROMRead(dataout.buf, 0x0, sizeof(dataout.buf));

	uint8_t len = sizeof(floatArray);

	EEPROMProgram(floatArray, 0x0, sizeof(floatArray));

	EEPROMRead(readArray, 0x0, sizeof(readArray));

	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x04);
	
	while(1)
	{
	}
}

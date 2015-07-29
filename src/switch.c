#include "switch.h"

// GENERAL SWITCH FUNCTIONS
void switchesInit(SwitchData_t sw[3]) {
	initSwitch(SYSCTL_PERIPH_GPIOF, GPIO_PORTF_BASE, GPIO_PIN_4, &sw[0]);
	initSwitch(SYSCTL_PERIPH_GPIOB, GPIO_PORTB_BASE, GPIO_PIN_0, &sw[1]);
	initSwitch(SYSCTL_PERIPH_GPIOB, GPIO_PORTB_BASE, GPIO_PIN_1, &sw[2]);
}

void switchesUpdate(SwitchData_t sw[3]) {
	int i;
	for (i = 0; i < 3; ++i) {
		readSwitch(&sw[i]);
		driveSwitch(&sw[i], sw[i].readState);
	}
}

/**************** Utility Functions for Lighted 3-pos Switch ******************/
// Initilializes the 3-position, lighted switch
void initSwitch(uint32_t periph, uint32_t base, uint32_t pin,
				SwitchData_t *sData)
{
	sData->periph = periph;
	sData->portBase = base;
	sData->pinNum = pin;

	SysCtlPeripheralEnable(sData->periph);
	GPIOPinTypeGPIOInput(sData->portBase, sData->pinNum);
	GPIOPadConfigSet(sData->portBase, sData->pinNum, GPIO_STRENGTH_2MA,
					 GPIO_PIN_TYPE_STD_WPU);

	SysCtlDelay(10); // wait a few clock cycles for the switch signal to settle.

	// Sample the port with mask
	sData->readState = GPIOPinRead(sData->portBase, sData->pinNum) ? 1 : 0;
	sData->driveState = sData->readState;
	GPIOPinTypeGPIOOutput(sData->portBase, sData->pinNum);

	uint8_t mask = sData->driveState ? sData->pinNum : 0;
	GPIOPinWrite(sData->portBase, sData->pinNum, mask);

	return;
}

// Reads the switch position
void readSwitch(SwitchData_t *sData)
{
	GPIOPinTypeGPIOInput(sData->portBase, sData->pinNum);// Set GPIO to input
	GPIOPadConfigSet(sData->portBase, sData->pinNum, GPIO_STRENGTH_2MA,
					 GPIO_PIN_TYPE_STD_WPU);// I may not need this

	SysCtlDelay(5);	// wait a few clock cycles for the switch signal to settle.

	// Sample the port with mask
	sData->readState = GPIOPinRead(sData->portBase, sData->pinNum) ? 1 : 0;
	GPIOPinTypeGPIOOutput(sData->portBase, sData->pinNum);

	uint8_t mask = sData->driveState ? sData->pinNum : 0;
	GPIOPinWrite(sData->portBase, sData->pinNum, mask);

	return;
}

// Drives the switch's LED
void driveSwitch(SwitchData_t *sData, uint8_t direction)
{
	sData->driveState = direction;
	uint8_t mask = sData->driveState ? sData->pinNum : 0;
	GPIOPinWrite(sData->portBase, sData->pinNum, mask);

	return;
}

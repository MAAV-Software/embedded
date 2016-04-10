/*
 * Switch.cpp
 *
 *  Created on: Apr 9, 2016
 *      Author: Sasawat
 */

#include "Switch.hpp"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

ThreeSwitch::ThreeSwitch(uint32_t per, uint32_t base, uint32_t pin)
{
    periph = per;
    portBase = base;
    pinNum = pin;

    ROM_SysCtlPeripheralEnable(periph);
    ROM_GPIOPinTypeGPIOInput(portBase, pinNum);
    ROM_GPIOPadConfigSet(portBase, pinNum, GPIO_STRENGTH_2MA,
            GPIO_PIN_TYPE_STD_WPU);

    ROM_SysCtlDelay(10); // wait a few clock cycles for the switch signal to settle.

    // Sample the port with mask
    readState = ROM_GPIOPinRead(portBase, pinNum) ? 1 : 0;
    driveState = readState;
    ROM_GPIOPinTypeGPIOOutput(portBase, pinNum);

    uint8_t mask = driveState ? pinNum : 0;
    ROM_GPIOPinWrite(portBase, pinNum, mask);

    return;
}

void ThreeSwitch::read()
{
    ROM_GPIOPinTypeGPIOInput(portBase, pinNum);// Set GPIO to input
    ROM_GPIOPadConfigSet(portBase, pinNum, GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPU);// I may not need this

    ROM_SysCtlDelay(5); // wait a few clock cycles for the switch signal to settle.

    // Sample the port with mask
    readState = ROM_GPIOPinRead(portBase, pinNum) ? 1 : 0;
    ROM_GPIOPinTypeGPIOOutput(portBase, pinNum);

    uint8_t mask = driveState ? pinNum : 0;
    ROM_GPIOPinWrite(portBase, pinNum, mask);

    return;
}

void ThreeSwitch::drive(uint8_t value)
{
    driveState = value;
    uint8_t mask = driveState ? pinNum : 0;
    ROM_GPIOPinWrite(portBase, pinNum, mask);

    return;
}

void ThreeSwitch::update()
{
    read();
    drive(readState);
}

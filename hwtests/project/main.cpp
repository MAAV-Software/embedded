/*
 * main.cpp
 *
 * Main function for Hardware Test Cases
 *
 *____    __    ____  ___      .______      .__   __.  __  .__   __.   _______
 *\   \  /  \  /   / /   \     |   _  \     |  \ |  | |  | |  \ |  |  /  _____|
 * \   \/    \/   / /  ^  \    |  |_)  |    |   \|  | |  | |   \|  | |  |  __
 *  \            / /  /_\  \   |      /     |  . `  | |  | |  . `  | |  | |_ |
 *   \    /\    / /  _____  \  |  |\  \----.|  |\   | |  | |  |\   | |  |__| |
 *    \__/  \__/ /__/     \__\ | _| `._____||__| \__| |__| |__| \__|  \______|
 *
 *      _                     _                         _ _  __
 *   __| | ___    _ __   ___ | |_   _ __ ___   ___   __| (_)/ _|_   _
 *  / _` |/ _ \  | '_ \ / _ \| __| | '_ ` _ \ / _ \ / _` | | |_| | | |
 * | (_| | (_) | | | | | (_) | |_  | | | | | | (_) | (_| | |  _| |_| |
 *  \__,_|\___/  |_| |_|\___/ \__| |_| |_| |_|\___/ \__,_|_|_|  \__, |
 *                                                              |___/
 *
 * THIS FILE IS AUTOMATICALLY GENERATED
 *
 * YOUR CHANGES WILL BE OVERWRITTEN
 *
 *      Author: Sasawat Prankprakma
 *        Date: Oct 12, 2016
 */

#include <stdint.h>
#include <cstdlib>

#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c123gh6pm.h"

#include "driverlib/gpio.h"
#include "driverlib/fpu.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/eeprom.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/interrupt.h"

#include "general.h"
#include "time_util.h"

#include "TestFunction.hpp"

int main(void) {

    MAP_FPULazyStackingEnable();
    MAP_FPUEnable();

    MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    time_init(SYSCTL_PERIPH_TIMER1, SYSCLOCK, TIMER1_BASE, INT_TIMER1A);

    TestFunction test;
    test.run();
	
	return 0;
}

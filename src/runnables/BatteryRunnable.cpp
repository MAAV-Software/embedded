/*
 * BatteryRunnable.cpp
 *
 *  Created on: Jul 30, 2015
 *      Author: Zhengjie
 */

#include "runnables/BatteryRunnable.hpp"
#include "Battery.hpp"
#include "BatteryHw.hpp"
#include "LED.h"
#include "time_util.h"

BatteryRunnable::BatteryRunnable(ProgramState *pState): state(pState)
 {
	ConfigADC(SYSCTL_PERIPH_ADC0, ADC0_BASE, SYSCTL_PERIPH_GPIOB, GPIO_PORTB_BASE, GPIO_PIN_4, ADC_CTL_CH10);
 }


void BatteryRunnable::run()
{
	state->battery->update(getADC(ADC0_BASE));
	if (state->battery->isLow())
	{
		Toggle_LED(RED_LED, SYSCLOCK/1000/2);
	}
}


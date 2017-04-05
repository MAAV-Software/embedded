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
#include "messaging/emergency_t.h"
#include "messaging/DataLink.hpp"
#include "Vehicle.hpp"

/**
 * @brief configures the ADC for the battery.
 * @param pState input program state
 */
BatteryRunnable::BatteryRunnable(ProgramState *pState): state(pState)
{
	// THIS IS FOR 2015 SIGNAL BOARD
	//ConfigADC(SYSCTL_PERIPH_ADC0, ADC0_BASE, SYSCTL_PERIPH_GPIOB, GPIO_PORTB_BASE, GPIO_PIN_4, ADC_CTL_CH10);

	// THIS IS FOR 2016 SIGNAL BOARD
	ConfigADC(SYSCTL_PERIPH_ADC0, ADC0_BASE, SYSCTL_PERIPH_GPIOE, GPIO_PORTE_BASE, GPIO_PIN_0, ADC_CTL_CH3);
}

/**
 * @brief checks the state of the battery.
 */
void BatteryRunnable::run()
{
	state->battery->update(getADC(ADC0_BASE));
	if (state->battery->isLow())
	{
		//Toggle_LED(RED_LED, SYSCLOCK/1000/2);

		emergency_t msg;
		msg.status = (int8_t)EMERGENCY_T_LOW_BATTERY;
		state->dLink->send(&msg);

		// TODO: land

	}
}


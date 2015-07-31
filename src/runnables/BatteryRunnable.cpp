#include "runnables/BatteryRunnable.hpp"
#include "Battery.hpp"
#include "BatteryHw.hpp"

BatteryRunnable::BatteryRunnable(ProgramState *pState): state(pState)
 {
	ConfigADC(SYSCTL_PERIPH_ADC0, ADC0_BASE, SYSCTL_PERIPH_GPIOB, GPIO_PORTB_BASE, GPIO_PIN_4, ADC_CTL_CH10);
 }


void BatteryRunnable::run()
{
	state->battery->update(getADC(ADC0_BASE));
}


#include "BatteryHw.hpp"

void ConfigADC(const uint32_t sysctlPeriphADC, const uint32_t adcBase,  const uint32_t sysctlPeriphGPIO, const uint32_t gpioPortBase, const uint8_t gpioADCPin, const uint8_t adcChannel)
{

		// ADC0
		ROM_SysCtlPeripheralEnable(sysctlPeriphADC);

		// PortB 4 is ADC channel 10;
		ROM_SysCtlPeripheralEnable(sysctlPeriphGPIO);
		ROM_GPIOPinTypeADC(gpioPortBase, gpioADCPin);

		// SEQ 1 has 4 steps, priority 0
		ROM_ADCSequenceConfigure(adcBase, 1, ADC_TRIGGER_PROCESSOR, 0);

		// Configure each step to sample on channel 10
		ROM_ADCSequenceStepConfigure(adcBase, 1, 0, adcChannel);
		ROM_ADCSequenceStepConfigure(adcBase, 1, 1, adcChannel);
		ROM_ADCSequenceStepConfigure(adcBase, 1, 2, adcChannel);
		ROM_ADCSequenceStepConfigure(adcBase, 1, 3, adcChannel|ADC_CTL_IE|ADC_CTL_END);
		ROM_ADCSequenceEnable(adcBase, 1);

}

// return the value between [0 2^12], reference voltage 3.3f
uint32_t getADC(const uint32_t adcBase)
{
	ROM_ADCIntClear(adcBase, 1);
	ROM_ADCProcessorTrigger(adcBase, 1);

   while(!ROM_ADCIntStatus(adcBase, 1, false));

   uint32_t ADCValue[4];
   ROM_ADCSequenceDataGet(adcBase, 1, ADCValue);
   return  ((ADCValue[0] + ADCValue[1] + ADCValue[2] + ADCValue[3]) / 4) ;
}

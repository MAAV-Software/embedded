#include <stdint.h>

#include "runnables/ImuRunnable.hpp"
#include "ImuHw.hpp"

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"

ImuRunnable::ImuRunnable(ProgramState *pState) : imu(pState->imu)
{
	// may want to move to main
	imuUartConfig(SYSCTL_PERIPH_UART1, SYSCTL_PERIPH_GPIOC, GPIO_PC4_U1RX,
				  GPIO_PC5_U1TX, GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_5,
				  UART1_BASE, INT_UART1);
}

void ImuRunnable::run()
{
	uint32_t getTime = TimerValueGet(TIMER4_BASE, TIMER_A);
	float sysClock = (float)SysCtlClockGet();

	// Run at 100Hz 10ms
	if (!imuDone && ((getTime - imuTime) > (sysClock / 1000.0 * 10.0)))
	{
		imuTime = getTime;
		imuUartSend(&imuCmd, 1);
	}

	if (imuDone)
	{
		imu->parse(imuRawFinal);
		imuDone = false;
	}
}

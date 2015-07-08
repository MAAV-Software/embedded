#include "runnables/ImuRunnable.hpp"

#include "Imu_hw.hpp"
//#include "Imu.hpp"

#include "time_util.h"

ImuRunnable::ImuRunnable(ProgramState *pState)
	:imu(pState->imu)
{
//	imu_uart_config_sys_clock();
	imu_uart_config_LED();
	imu_uart_config_uart();
//    imu_uart_timer_init();

	uint32_t g_one_sec = SysCtlClockGet()/3;

    UARTprintf("IMU_Testing\n\r");
    int i = 0;
    for (i = 0; i < 3; i++)
    {
    	UARTprintf("IMU_Initializing CountDown:%u\n\r",3-i);
    	SysCtlDelay(g_one_sec);
    	imu_uart_toggle_LED(GREEN_LED, g_one_sec/20);
    }
    UARTprintf("IMU_Ready\n\r");
	imuDone = false;
}

ImuRunnable::~ImuRunnable(){}

void ImuRunnable::run(void)
{
	uint32_t ui32_gettime = TimerValueGet(TIMER4_BASE, TIMER_A);
	uint32_t sysclock = SysCtlClockGet();

	// Run at 100Hz 10ms
	if (!imuDone && ((ui32_gettime - imu_time) > (float)sysclock/1000.0*10.0))
	{
		imu_time = ui32_gettime;
		imu_uart_send(IMU_UART_BASE, &imuCmd, 1);
	}

	if (imuDone)
	{
		imu->parse(imuRawFinal);

//		UARTprintf("Timer(s):%u\tRoll*1000:%d\tPitch*1000:%d\tYaw*1000:%d \n\r", imu->getTimer(),(int)(imu->getRoll()*1000),(int)(imu->getPitch()*1000),(int)(imu->getYaw()*1000));
		imuDone = false;
	}
}

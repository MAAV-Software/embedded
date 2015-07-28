//Predefined Symbol: PART_TM4C123GH6PM, UART_BUFFERED
//Linked Resources: Path Variable: TIVAWARE_INSTALL
//Build: Variable: Directory: TIVAWARE_INSTALL
//Build->Include Option: Path: ${TIVAWARE_INSTALL}

#include "Imu.hpp"
#include "Imu_hw.hpp"

uint32_t ui32_dt;
char timervalue[13];

// 79-Bytes Command - 0xCC
static uint8_t imuCmd = (uint8_t)ALL_VAL_CMD;

//void ItoS(uint32_t integer,char buffer[], uint8_t len);
extern bool imuDone;

//****************************************************************************************
int main(void)
{

	Imu imu;

	imu_uart_config_sys_clock();

	imu_uart_config_LED();

    imu_uart_config_uart();

    UARTprintf("IMU_Test\n\r");

    imu_uart_timer_init();

    uint32_t g_one_sec = SysCtlClockGet()/3;

    UARTprintf("IMU_Test one sec:%u\n\r", g_one_sec);
    // Wait sometime(3 sec) for the initialization of the board
    int i = 0;
    for (i = 0; i < 3; i++)
    {
    	SysCtlDelay(g_one_sec);
    	imu_uart_toggle_LED(GREEN_LED, g_one_sec/20);
    }

	imuDone = false;

    // Main application loop.
    while(1)
    {

    	uint32_t ui32_gettime = TimerValueGet(TIMER0_BASE, TIMER_A);

    	// Run at 100Hz
    	if (!imuDone && ((ui32_gettime - imu_time) > 500000))
    	{
    		imu_time = ui32_gettime;
    		imu_uart_send(IMU_UART_BASE, &imuCmd, 1);
    	}

    	if (imuDone)
    	{
    		imu.parse(imuRawFinal);

    		// Debug output
//    		char output[100];
//
//			int out_length = snprintf(output, 100, "Timer(s):%u\tRoll*1000:%d\tPitch*1000:%d\tYaw*1000:%d \n\r", imu.getTimer(),(int)(imu.getRoll()*1000),(int)(imu.getPitch()*1000),(int)(imu.getYaw()*1000));
//
//    		imu_uart_send(PC_UART_BASE, (uint8_t *)output, out_length);
    		UARTprintf("Timer(s):%u\tRoll*1000:%d\tPitch*1000:%d\tYaw*1000:%d \n\r", imu.getTimer(),(int)(imu.getRoll()*1000),(int)(imu.getPitch()*1000),(int)(imu.getYaw()*1000));
    		imuDone = false;
    	}


    }

}


//Predefined Symbol: PART_TM4C123GH6PM, UART_BUFFERED
//Linked Resources: Path Variable: TIVAWARE_INSTALL
//Build: Variable: Directory: TIVAWARE_INSTALL
//Build->Include Option: Path: ${TIVAWARE_INSTALL}

#include "imu.hpp"
#include "imu_hw.hpp"

uint32_t ui32_current_time;
uint32_t ui32_gettime = 0;
uint32_t ui32_dt;
char timervalue[13];

// 79-Bytes Command - 0xCC
static uint8_t imuCmd = (uint8_t)ALL_VAL_CMD;

//void ItoS(uint32_t integer,char buffer[], uint8_t len);
bool imuReady;

//****************************************************************************************
int main(void)
{
	imuReady = false;
	
	Imu imu;

	imu_uart_config_sys_clock();

	imu_uart_config_LED();

    imu_uart_config_uart();

    imu_uart_timer_init();

    uint32_t g_one_sec = SysCtlClockGet()/3;

    // Wait sometime(3 sec) for the initialization of the board
    int i = 0;
    for (i = 0; i < 3; i++)
    {
    	SysCtlDelay(g_one_sec);
    	imu_uart_toggle_LED(GREEN_LED, g_one_sec/10);
    }

    //uint32_t printTime = 0;

    // Main application loop.
    while(1)
    {

    	uint32_t ui32_gettime = TimerValueGet(TIMER0_BASE, TIMER_A);
		//ui32_dt = ui32_gettime - ui32_current_time;

		//if(ui32_dt > 4290000000)
			//ui32_dt = 4294967295 - ui32_dt;

    	// Run at 100Hz
    	uint32_t cmdDt = ui32_gettime - ui32_current_time;
    	if (cmdDt > 4290000000) cmdDt = 4294967295 - cmdDt;
    	if (!imuReady && (cmdDt > 500000))
    	{
    		imu_uart_send(IMU_UART_BASE, &imuCmd, 1);
    		ui32_current_time = ui32_gettime;
    	}

    	if (imuReady)
    	{
    		imu.parse(imuRawFinal);

    		// Debug output
//    		char output[200];
//    		int timer = snprintf(output, 200, "Timer:%u \n\r", imu.getTimer());
//
//    		imu_uart_send(PC_UART_BASE, (uint8_t *)output, 200);

    		imuReady = false;
    		//imuActive = false;

//        	imu_uart_toggle_LED(RED_LED, g_one_sec/20000);
    	}

//    	uint32_t printDt = ui32_gettime - printTime;
//    	if (printDt > 4290000000) printDt = 4294967295 - printDt;
//    	if (printDt > 25000000)
//    	{
//    		printTime = ui32_gettime;
//
//    		// Debug output
//    		char output[200];
//    		int timer = snprintf(output, 200, "Timer:%u \n\r", imu.getTimer());
//
//    		imu_uart_send(PC_UART_BASE, (uint8_t *)output, 200);
//
//    	}

    }

}


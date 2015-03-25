//Predefined Symbol: PART_TM4C123GH6PM, UART_BUFFERED
//Linked Resources: Path Variable: TIVAWARE_INSTALL
//Build: Variable: Directory: TIVAWARE_INSTALL
//Build->Include Option: Path: ${TIVAWARE_INSTALL}

#include "imu_uart.h"

//****************************************************************************************
int main(void)
{
	g_IMU_Status = 1;
	
	// Might not be callled if it has already been called somewhere else
	imu_uart_config_sys_clock();

    imu_uart_config_LED();

//    imu_uart_config_SW();

    imu_uart_config_uart();

    g_one_sec = SysCtlClockGet()/3;

    // Wait sometime(3 sec) for the initialization of the board
    // Toggle Green Led 1 sec for 3 times
    int i = 0;
    for (i = 0; i < 3; i++)
    {
    	imu_uart_toggle_LED(GREEN_LED, g_one_sec);
    }

    // Main application loop.
    while(1)
    {
    	if (g_IMU_Status)
    	{

			// Toggle BLUE LED, reading data
//			imu_uart_toggle_LED(BLUE_LED, g_one_sec/10000);

			// ACC&AngRate Command
			imu_uart_send(UART1_BASE, (uint8_t *)g_IMU_Address_Sent, 1);

			char output[20];

			// Debug and test
			// Print out Timmer
			int out_length = snprintf(output, 20, "Timer:%u \n\r", imu_uart_getTimer());

			// Print out Roll data
		//	int out_length = ltoa((int)(imu_uart_getRoll()*1000), output);

//			imu_uart_send(UART0_BASE, (uint8_t*)output, out_length);
		//	imu_uart_send(UART0_BASE, "\r\n", 2);

    	}
    	else
    	{
    		// Toggle RED LED waiting for wake_up signal
    		imu_uart_toggle_LED(RED_LED, g_one_sec/2);
    	}
    }

}

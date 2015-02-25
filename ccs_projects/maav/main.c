/*
 * main.c
 *
 * Main flight control program for MAAV. Iterfaces with Atom, RC controllers,
 * and DJI. Executes outerlook position control of DJI (which handles inner
 * loop attitude control).
 *
 *      Author: Sajan Patel, Jonathan Kurzer, Sasawat Prankprakma, Clark Zhang
 *        Date: Feb 24, 2015
 *
 */
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

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
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"

#include "utils/uartstdio.h"

#include "PPM.h"
#include "px4_i2c.h"
#include "servoIn.h"
#include "time_util.h"
#include "px4_kalman.h"
#include "dof.h"
#include "quad_ctrl.h"

#include "messaging/data_link.h"

#include "utility.h"

bool px4_can_transmit = true;

////////////////////////////// MAIN FUNCTION ///////////////////////////////////
int main(void)
{
	/*
	 *  Set system clock to 80Mhz.
	 *  Note that SysCtlClockGet() has a bug for this frequency.
	 *  Use the #defined constant "SYSCLOCK" from utility.h instead.
	 */
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
				   SYSCTL_XTAL_16MHZ);

	// Init the LEDs on the Launchpad for debugging and init them to off
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);

	// initialize switches
	SwitchData_t sw[3];
	initSwitch(SYSCTL_PERIPH_GPIOB, GPIO_PORTB_BASE, GPIO_PIN_1, &sw[0]);
	initSwitch(SYSCTL_PERIPH_GPIOB, GPIO_PORTB_BASE, GPIO_PIN_2, &sw[1]);
	initSwitch(SYSCTL_PERIPH_GPIOB, GPIO_PORTB_BASE, GPIO_PIN_3, &sw[2]);

	// Intialize timer for PPM, PPM, and timer 4 for servo (RC) input
	time_init(SYSCTL_PERIPH_TIMER1, SYSCLOCK, TIMER1_BASE, INT_TIMER1A);	// Chose any open timer
	PPM_init(SYSCTL_PERIPH_TIMER2, SYSCLOCK, TIMER2_BASE, INT_TIMER2A,		// Chose any open timer
			 GPIO_PORTB_BASE, GPIO_PIN_6, 4);								// Chose any open port/pin
	servoIn_init(SYSCTL_PERIPH_TIMER4, TIMER4_BASE); // Chose timer4 until encapsulated
	servoIn_attachPin();

	// Init PX4 on I2C Ch 3 on Port D Pins 0-3
	init_px4_i2c(SYSCTL_PERIPH_I2C3, SYSCTL_PERIPH_GPIOD, SYSCTL_PERIPH_GPIOD,
				 SYSCLOCK, I2C3_BASE, GPIO_PORTD_BASE, GPIO_PORTD_BASE,
				 GPIO_PIN_0, GPIO_PIN_1, GPIO_PD0_I2C3SCL, GPIO_PD1_I2C3SDA);

	// Turn on Floating point hardware
	FPULazyStackingEnable();
	FPUEnable();

	// Enable EEPROM
	SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
	EEPROMInit();

	// Enable Interrupts
	IntMasterEnable();

	// Initialize messaging system structs for data link
	target_t *msg_target = (target_t*)malloc(sizeof(target_t));
	position_t *msg_position = (position_t*)malloc(sizeof(position_t));
	tuning_t *msg_tuning = (tuning_t*)malloc(sizeof(tuning_t));
	int32_t last_target_time = 0;
	int32_t last_position_time = 0;
	int32_t last_tuning_time = 0;
	feedback_t feedback_send;

	// Call init function
	data_link_init(msg_position, msg_target, msg_tuning);

	// Set up UART comms to computer terminal
	ConfigureUART();

	// Enable UART Receive Timeout and Receive interrupt for UART comms to computer
	IntEnable(INT_UART0);
	UARTIntEnable(UART0_BASE, UART_INT_RT | UART_INT_RX);

	// Init Kalman Filter
	kalman_t filter_data;
	kalman_t *filter = &filter_data;
    kalman_create(filter);
	uint16_t oldFrameCount = 0;

	/* Create quad_ctrl and intialize it. (NOT WITH DYNAMIC MEMORY) */
	quad_ctrl_t qc;
	qc_init(&qc);
	qc.ctrl_mode = RC_CTRL;

	// Initialize the PID gains [Kp, Ki, Kd] for all arrays
	float *xy_valueGains = qc.xyzh[X_AXIS].value_gains;
	float xy_rateGains[] = {1.00, 0.00, 0.01};
	float z_valueGains[] = {1.00, 0.00, 0.01};
	float *z_rateGains = qc.xyzh[Z_AXIS].rate_gains;

	readSwitch(&sw[0]);
	if(sw[0].readState) copyGains(&qc);

	//float *yaw_valueGains = qc.xyzh[YAW].value_gains;
	//float *yaw_rateGains = qc.xyzh[YAW].rate_gains;

	// Initialize setpt array [x, y, z, yaw, x_dot, y_dot, z_dot, yaw_dot]
	float setpoints[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	// Initialize feedback array [x, y, z, yaw, x_dot, y_dot, z_dot, yaw_dot]
	float feedback[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	uint32_t loopTime = 0;
	uint32_t switchUpdateTime = 0;
	uint32_t update_DJI_time = 0;
	uint32_t modeCheckTime = 0;
	uint32_t writeEepromTime = 0;
	uint32_t gainCheckTime = 0;
	uint32_t update_PX4_time = 0;
	uint32_t test_PX4_time = 0;
	uint32_t update_setPoints_time = 0;
	uint32_t lastFreshDataTime = 0;
	uint32_t process_data_link_data_time = 0;
	uint8_t  mode = 3;	// defalut to RC mode
	bool px4_can_transmit = true; // flag for PX4 transmission

	SysCtlDelay(SYSCLOCK);	// about 3 seconds.  Required for DJI startup

	for (;;) // master loop
	{
		loopTime = millis(); // get current time

		//TODO: Determine how often we want to process newly received data
		//There is no rush since the dumping of fifo to ring buffer is done in isr
		//But more often processing means faster response to messages
		//This has no effect on sending whatsoever
		if (loopTime-process_data_link_data_time > 10)
			data_link_process_incoming();

		if (msg_target->timestamp > last_target_time)
		{
			last_target_time = msg_target->timestamp;
			// TODO:Handle target message takeoff, land
			// Currently done: set setpoint
			targetMessageQuadCtrlChangesHandler(&qc, msg_target);
		}

		if (msg_tuning->timestamp > last_tuning_time)
		{
			last_tuning_time = msg_tuning->timestamp;
			// TODO:Handle tuning message takeoff, land
			// Currently done: set kpid, set ratekpid, set setpoint and rate setpoint
			// Handle the changing of gains (if any) in the tuning message
			tuningMessageQuadCtrlChangesHandler(&qc, msg_tuning);
		}

		if (msg_position->timestamp > last_position_time)
		{
			last_position_time = msg_position->timestamp;
			// TODO:Handle position message
			// Someone that knows the insides of the filter should probably do this part
		}

		// check lighted switches
		if ((loopTime - switchUpdateTime) > 10)
		{
			switchUpdateTime = loopTime;
			int i;
			for (i = 0; i < 3; ++i) readSwitch(&sw[i]);
			driveSwitch(&sw[0], sw[0].readState);
			driveSwitch(&sw[1], sw[1].readState);
			driveSwitch(&sw[2], sw[2].readState);
		} // end switch check


		// Check controller mode from RC Kill Switch switch
		if ((loopTime - modeCheckTime) > 100)
		{
			modeCheckTime = loopTime;
			mode = automomousMode(servoIn_getPulse(RC_CHAN5)) ? 3 :
					pulseUpperThird(servoIn_getPulse(KILL_CHAN5)) ? 1 : 2;
			switch (mode)
			{
				case 1: GPIOPinWrite(GPIO_PORTF_BASE, RED_LED | GREEN_LED |
									 BLUE_LED, RED_LED);
					break;
				case 2: GPIOPinWrite(GPIO_PORTF_BASE, RED_LED | GREEN_LED |
									 BLUE_LED, BLUE_LED);
					break;
				case 3: GPIOPinWrite(GPIO_PORTF_BASE, RED_LED | GREEN_LED |
									 BLUE_LED, GREEN_LED);
					break;
			}
		} // end controller mode check

		// Every now and then, log the gains to EEPROM
		if((loopTime - writeEepromTime) > 1000)
		{
			writeEepromTime = loopTime;
			recordGains(&qc);
		}

		// Update controller gains
		if((loopTime - gainCheckTime) > 100)
		{
			gainCheckTime = loopTime;
//			char buffer[100];
//			uint32_t len = snprintf(buffer, 1000,
//					"%f,\t%f,\t%f,\t%f\n",
//					xy_rateGains[Kp],
//					xy_rateGains[Kd],
//					z_valueGains[Kp],
//					z_valueGains[Kd]);
//			UARTwrite(buffer, len);

			if (mode == 1) // XY-Rate gains
			{
				if (      pulseUpperThird(servoIn_getPulse(KILL_CHAN1))) xy_rateGains[Kd] *= 1.01;
				else if ( pulseLowerThird(servoIn_getPulse(KILL_CHAN1))) xy_rateGains[Kd] *= 0.99;
				if (      pulseUpperThird(servoIn_getPulse(KILL_CHAN2))) xy_rateGains[Kp] *= 1.01;
				else if ( pulseLowerThird(servoIn_getPulse(KILL_CHAN2))) xy_rateGains[Kp] *= 0.99;

				dof_set_gains(&(qc.xyzh[X_AXIS]), xy_valueGains, xy_rateGains);
				dof_set_gains(&(qc.xyzh[Y_AXIS]), xy_valueGains, xy_rateGains);

			}
			else if (mode == 2) // Z-Value gains
			{
				if (      pulseUpperThird(servoIn_getPulse(KILL_CHAN1))) z_valueGains[Kd] *= 1.01;
				else if ( pulseLowerThird(servoIn_getPulse(KILL_CHAN1))) z_valueGains[Kd] *= 0.99;
				if (      pulseUpperThird(servoIn_getPulse(KILL_CHAN2))) z_valueGains[Kp] *= 1.01;
				else if ( pulseLowerThird(servoIn_getPulse(KILL_CHAN2))) z_valueGains[Kp] *= 0.99;

				dof_set_gains(&(qc.xyzh[Z_AXIS]), z_valueGains, z_rateGains);
			}
		} // End gains update

		// Update Setpoints: setpoints array [x, y, z, yaw, x_dot, y_dot, z_dot, yaw_dot]
		if (loopTime-update_setPoints_time > 20)
		{
			update_setPoints_time = loopTime;

			setpoints[5] = ms2XY_rate(pulse2ms(servoIn_getPulse(RC_CHAN1)));	// Y Rate
			setpoints[4] = ms2XY_rate(pulse2ms(servoIn_getPulse(RC_CHAN2)));	// X Rate
			setpoints[2] = ms2height(pulse2ms(servoIn_getPulse(RC_CHAN3)));		// Z Absolute
			//setpoints[7] = pulse2ms(servoIn_getPulse(RC_CHAN4));	// Yaw Rate	TODO Add this back in later
			setpoints[0] = setpoints[1] = setpoints[3] = setpoints[6] = setpoints[7] = 0;

			if (qc.xyzh[Z_AXIS].Uval > 0)	 	driveSwitch(&sw[0], 1);
			else 								driveSwitch(&sw[0], 0);
			if (qc.xyzh[Z_AXIS].setpt[0] >1.0)	driveSwitch(&sw[2], 1);
			else								driveSwitch(&sw[2], 0);

			qc_setSetpt(&qc, setpoints, timestamp_now());
		} // End Setpoint Update

		// Poll PX4 for new measurement
		if (((loopTime - update_PX4_time) > 10) && (px4_can_transmit == true))
		{
			update_PX4_time = loopTime;
			initiate_PX4_transmit();
			px4_can_transmit = false;
			driveSwitch(&sw[1], 1);
		} // End PX4 Polling

		// Test for I2C race condition during transmission
		if ((loopTime - test_PX4_time) > 25000)
		{
			test_PX4_time = loopTime;

			uint16_t frameCount = px4_i2c_get_frame_count();

			// I2C Failure Recovery
			if ((loopTime - lastFreshDataTime > 50000) || (frameCount == 65535)
				|| (frameCount == 0))
			{
				driveSwitch(&sw[1], 1);
				UARTprintf("\n\nI2C_fail\n\n");

				SysCtlPeripheralReset(SYSCTL_PERIPH_I2C3);
				SysCtlDelay(100);	// wait a few clock cycles for the switch signal to settle.

				// re-init PX4 comm
				init_px4_i2c(SYSCTL_PERIPH_I2C3, SYSCTL_PERIPH_GPIOD,
							 SYSCTL_PERIPH_GPIOD, SYSCLOCK, I2C3_BASE,
							 GPIO_PORTD_BASE, GPIO_PORTD_BASE, GPIO_PIN_0,
							 GPIO_PIN_1, GPIO_PD0_I2C3SCL, GPIO_PD1_I2C3SDA);
			}
            else driveSwitch(&sw[1], 0);

			char buffer[100];
			uint32_t len = snprintf(buffer, 100, "%d\n",
									px4_i2c_get_frame_count());
			UARTwrite(buffer, len);
		} // End PX4 race condition test

		/*
		 * Get PX4 data and feed into Kalman Filter.
		 */
        if (px4_i2c_dataFresh())
        {
        	lastFreshDataTime = loopTime;

        	uint16_t frameCount = px4_i2c_get_frame_count();
            if (frameCount != oldFrameCount)
            {
                kalman_process_data(filter,
                                    px4_i2c_get_flow_comp_m_x(),
                                    px4_i2c_get_flow_comp_m_y(),
                                    px4_i2c_getHeight(),
                                    px4_i2c_get_gyro_z_rate(),
                                    px4_i2c_get_gyro_range(),
                                    px4_i2c_get_qual(),
                                    px4_i2c_getTimestep(),
                                    timestamp_now());

                if ((frameCount != 65535) && (frameCount != 0))
                	sendToSerialPort(filter, frameCount);
            }

            oldFrameCount = frameCount;
            px4_i2c_makeDataStale();
            px4_can_transmit = true;
        } // End Kalman Process Data

        /*
         * On every iteration, do Kalman Predict and Kalman Correct steps by
         * calling kalman_update(). Extract data into feedback and set
         * qc state. Then run all PID with qc_runPID().
         */
        uint64_t tempTimestamp = timestamp_now();
        kalman_update(filter, tempTimestamp);

        feedback[0] = filter->x; 		// X from filter
        feedback[1] = filter->y; 		// Y from filter
        feedback[2] = filter->z; 		// Z from filter
        feedback[3] = filter->xdot; 	// Yaw from filter
        feedback[4] = filter->ydot; 	// X_dot from filter
        feedback[5] = filter->zdot; 	// Y_dot from filter
        feedback[6] = filter->yaw; 		// Z_dot from filter
        feedback[7] = filter->yawdot; 	// Yaw_dot from filter

        qc_setState(&qc, feedback, tempTimestamp);	// Send feedback to quad control
        qc_runPID(&qc);								// Run PID

        // Send updated signals to DJI
		if ((loopTime - update_DJI_time) > 10)
		{
			update_DJI_time = loopTime;

			if ((mode == 1) || (mode == 2)) // autonomous mode. do something smart
			{
				PPM_setPulse(0, servoIn_getPulse(RC_CHAN1));
				PPM_setPulse(1, servoIn_getPulse(RC_CHAN2));

				// convert Z Uval into PWM Pulse
				float zPulse = PID_XY_2ms(qc.xyzh[Z_AXIS].Uval);

				zPulse = (zPulse > 1.2) ? zPulse : 1.2;

				PPM_setPulse(2, ms2pulse(zPulse));	// Z control to DJI
				PPM_setPulse(3, servoIn_getPulse(RC_CHAN4));
			}
			else // RC passthrough.  Dump RC Data directly into the DJI
			{
				PPM_setPulse(0, servoIn_getPulse(RC_CHAN1));	// Y Accel
				PPM_setPulse(1, servoIn_getPulse(RC_CHAN2));	// X Accel
				PPM_setPulse(2, servoIn_getPulse(RC_CHAN3));	// Z Accel
				PPM_setPulse(3, servoIn_getPulse(RC_CHAN4));	// Yaw Rate
			}
		} // end DJI update
	} // end main loop
}
// End of File

/*
 * main.c
 */
#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/tm4c123gh6pm.h"

#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/fpu.h"
#include "driverlib/adc.h"

#include "time_util.h"
#include "PPM.h"
#include "servoIn.h"
#include "quad_ctrl.h"

#define SYSCLOCK 80000000
#define RED_LED   GPIO_PIN_1
#define GREEN_LED GPIO_PIN_3
#define BLUE_LED  GPIO_PIN_2

#define RC_CHAN1 GPIO_PORTA_BASE,2
#define RC_CHAN2 GPIO_PORTA_BASE,3
#define RC_CHAN3 GPIO_PORTA_BASE,5
#define RC_CHAN4 GPIO_PORTA_BASE,6
#define RC_CHAN5 GPIO_PORTA_BASE,7

#define KILL_CHAN1 GPIO_PORTE_BASE,4
#define KILL_CHAN2 GPIO_PORTE_BASE,5
#define KILL_CHAN4 GPIO_PORTE_BASE,1
#define KILL_CHAN5 GPIO_PORTE_BASE,2

#define MAX_TRACKER_SIZE 1000

enum PID_gains_enum {Kp, Ki, Kd};

bool pulseUpperThird(volatile uint32_t pulseWidth) {	// Is pulse longer than 1.66ms?
	return (pulseWidth > SYSCLOCK / 602) ? true : false;
}
bool pulseLowerThird(volatile uint32_t pulseWidth) {	// Is pulse shorter than 1.33ms?
	return (pulseWidth < SYSCLOCK / 752) ? true : false;
}
bool automomousMode(volatile uint32_t pulseWidth) {
	return(pulseLowerThird(pulseWidth));
}
float pulse2ms(uint32_t pulse) {
	static float one_ms = SYSCLOCK / 1000;
	return( (float)(pulse) / one_ms);
}
uint32_t ms2pulse(float ms) {
	static float one_ms = SYSCLOCK / 1000;
	return( (uint32_t)(ms * one_ms) );
}
float map(float x, float fromLow, float fromHigh, float toLow, float toHigh) {
	return (x - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}
float ms2height(float ms) {
	return(map(ms, 1.0, 2.0, 0.5, 1.5));
}

int main(void) {
	// Set system clock to 80Mhz.
	// Note that SysCtlClockGet() has a bug for this frequency.
	// Use the above #defined constant "SYSCLOCK" instead.
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN
				   | SYSCTL_XTAL_16MHZ);

	// Init the LEDs on the Launchpad for debugging and init them to off
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);

	/*
	uint32_t ui32ADC3Value[4];
	volatile uint32_t ui32HeightAvg;
	volatile uint32_t ui32TempValueC;
	volatile uint32_t ui32TempValueF;

	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	ADCHardwareOversampleConfigure(ADC0_BASE, 64);
	ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH3);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH3);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH3);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 3, ADC_CTL_CH3 | ADC_CTL_IE
							 | ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, 1);
	*/

	time_init(SYSCTL_PERIPH_TIMER1, SYSCLOCK, TIMER1_BASE, INT_TIMER1A);	// Chose any open timer
	PPM_init(SYSCTL_PERIPH_TIMER2, SYSCLOCK, TIMER2_BASE, INT_TIMER2A,		// Chose any open timer
			 GPIO_PORTB_BASE, GPIO_PIN_6, 4);								// Chose any open port/pin

	servoIn_init(SYSCTL_PERIPH_TIMER4, TIMER4_BASE);		// Chose timer4 until encapsulated
	servoIn_attachPin();									// TODO encapsulate this src

	// Turn on Floating point hardware
	FPULazyStackingEnable();
	FPUEnable();


	/* Create quad_ctrl and intialize it. (NOT WITH DYNAMIC MEMORY) */
	quad_ctrl_t qc;
	qc_init(&qc);
	qc.ctrl_mode = RC_CTRL;

	// Initialize the PID gains [Kp, Ki, Kd] for all arrays
	float *xy_valueGains = qc.xyzh[X_AXIS].value_gains;
	float xy_rateGains[] = {1.00, 0.00, 0.01};
	float z_valueGains[] = {1.00, 0.00, 0.01};
	float *z_rateGains = qc.xyzh[Z_AXIS].rate_gains;
	//float *yaw_valueGains = qc.xyzh[YAW].value_gains;
	//float *yaw_rateGains = qc.xyzh[YAW].rate_gains;

	// Initialize setpt array [x, y, z, yaw, x_dot, y_dot, z_dot, yaw_dot]
	float setpoints[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	// Initialize feedback array [x, y, z, yaw, x_dot, y_dot, z_dot, yaw_dot]
	float feedback[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	// simulation arrays
	float fb_tracker[MAX_TRACKER_SIZE], sp_tracker[MAX_TRACKER_SIZE];
	uint32_t track = 0, simtrack = 0;
	uint8_t sim = 1;

	for (simtrack = 0; simtrack < MAX_TRACKER_SIZE; ++simtrack)
	{
		if (simtrack < 500) sp_tracker[simtrack] = simtrack;
		else sp_tracker[simtrack] = 500;
	}

	uint32_t loopTime = 0;
	uint32_t update_DJI_time = 0;
	uint32_t modeCheckTime = 0;
	uint32_t gainCheckTime = 0;
	uint8_t  mode = 3;	// defalut to RC mode
	//int i;

	SysCtlDelay(SYSCLOCK);	// about 3 seconds.  Required for DJI startup

	// Do a zig-zag thing until I get the Pulse in data finished.
	//int32_t pos[] = {1000, 333, -333, -1000, 0, 0, 0};

	for(;;) {
		loopTime = millis();
		float ctrl_timestamp = (float)loopTime / 1000.0;
//		if(loopTime - newPX4_data_time > 100) {
//			newPX4_data_time = loopTime;
//
//			// Kallman update part
//			// Call Kalman update func
//		}
		if(loopTime-modeCheckTime > 100) {
			modeCheckTime = loopTime;
			mode = automomousMode(servoIn_getPulse(RC_CHAN5)) ? 3 : pulseUpperThird(servoIn_getPulse(KILL_CHAN5)) ? 1 : 2;
			switch(mode) {
			case(1):	GPIOPinWrite(GPIO_PORTF_BASE, RED_LED | GREEN_LED | BLUE_LED, RED_LED);		break;
			case(2):	GPIOPinWrite(GPIO_PORTF_BASE, RED_LED | GREEN_LED | BLUE_LED, BLUE_LED);	break;
			case(3):	GPIOPinWrite(GPIO_PORTF_BASE, RED_LED | GREEN_LED | BLUE_LED, GREEN_LED);	break;
			}
		}
		if(loopTime-gainCheckTime > 100) {
			gainCheckTime = loopTime;
			if(mode == 1) { // xy gain update
				if(      pulseUpperThird(servoIn_getPulse(KILL_CHAN1))) xy_rateGains[Kd] *= 1.01;
				else if( pulseLowerThird(servoIn_getPulse(KILL_CHAN1))) xy_rateGains[Kd] *= 0.99;
				if(      pulseUpperThird(servoIn_getPulse(KILL_CHAN2))) xy_rateGains[Kp] *= 1.01;
				else if( pulseLowerThird(servoIn_getPulse(KILL_CHAN2))) xy_rateGains[Kp] *= 0.99;

				dof_set_gains(&(qc.xyzh[X_AXIS]), xy_valueGains, xy_rateGains);
				dof_set_gains(&(qc.xyzh[Y_AXIS]), xy_valueGains, xy_rateGains);
			} else if(mode == 2) { // z gain update
				if(      pulseUpperThird(servoIn_getPulse(KILL_CHAN1))) z_valueGains[Kd] *= 1.01;
				else if( pulseLowerThird(servoIn_getPulse(KILL_CHAN1))) z_valueGains[Kd] *= 0.99;
				if(      pulseUpperThird(servoIn_getPulse(KILL_CHAN2))) z_valueGains[Kp] *= 1.01;
				else if( pulseLowerThird(servoIn_getPulse(KILL_CHAN2))) z_valueGains[Kp] *= 0.99;

				dof_set_gains(&(qc.xyzh[Z_AXIS]), z_valueGains, z_rateGains);
			}
		}
		if(loopTime - update_DJI_time > 10) {
			update_DJI_time = loopTime;
			//			for(i=0; i<4; i++) {
			//				pos[i] = (pos[i] + 10) % 4000;
			//				int32_t tmp = (pos[i] > 2000) ? 4000 - pos[i] : pos[i];
			//				PPM_setStickPos(i, tmp - 1000);
			//			}


			// Call Kalman filter to get updated X_accel, Y_accel, Z_accel
			// Kalman Predict part


			if(mode == 1 || mode == 2 || sim == 1){
				/*** Get Z Feedback Here ***/
				float Z_actual = 0;

				// pass through x, y, yaw channels to DJI
				PPM_setPulse(0, servoIn_getPulse(RC_CHAN1));
				PPM_setPulse(1, servoIn_getPulse(RC_CHAN2));
				//PPM_setPulse(2, servoIn_getPulse(RC_CHAN3));
				PPM_setPulse(3, servoIn_getPulse(RC_CHAN4));

				// get Z setpoint and send to qc
				//float Z_desired = ms2height(pulse2ms(servoIn_getPulse(RC_CHAN3)));
				//setpoints[3] = Z_desired;
				setpoints[3] = sp_tracker[track];
				qc_setSetpt(&qc, setpoints, ctrl_timestamp);

				// simulation feedback system
				fb_tracker[track] = qc.xyzh[Z_AXIS].Uval + qc.xyzh[Z_AXIS].state[0];

				// sim feedback tracker
				Z_actual = fb_tracker[track];

				// logic for wraparound of sim buffers
				track++;
				if (track == MAX_TRACKER_SIZE)
				{
					while(1)
					{
						// set breakpoint in here
					}
				}


				// send feedback to qc
				feedback[3] = Z_actual;
				qc_setState(&qc, feedback, ctrl_timestamp);

				// run PID
				qc_runPID(&qc);

				PPM_setPulse(2, ms2pulse(1.5 - qc.xyzh[Z_AXIS].Uval));

				//				ui32_PPM_Arr[2] = ms2pulse(1.5 - Z_PID_retVal);

			} else {  		// RC passthrough.  Dump RC Data directly into the DJI
				PPM_setPulse(0, servoIn_getPulse(RC_CHAN1));
				PPM_setPulse(1, servoIn_getPulse(RC_CHAN2));
				PPM_setPulse(2, servoIn_getPulse(RC_CHAN3));
				PPM_setPulse(3, servoIn_getPulse(RC_CHAN4));

				//				pulseMap_PortA[2] = &ui32_PPM_Arr[0];			// RC CH1
				//				pulseMap_PortA[3] = &ui32_PPM_Arr[1];			// RC CH2
				//				pulseMap_PortA[5] = &ui32_PPM_Arr[2];			// RC CH3
				//				pulseMap_PortA[6] = &ui32_PPM_Arr[3];			// RC CH4

			}
			//			servoIn_getPulse(RC_CHAN1);
			//			servoIn_getPulse(RC_CHAN2);
			//			servoIn_getPulse(RC_CHAN3);
			//			servoIn_getPulse(RC_CHAN4);
			//			servoIn_getPulse(RC_CHAN5);
			//
			//			servoIn_getPulse(KILL_CHAN1);
			//			servoIn_getPulse(KILL_CHAN2);
			//			servoIn_getPulse(KILL_CHAN4);
			//			servoIn_getPulse(KILL_CHAN5);
			//			PPM_setPulse(0, servoIn_getPulse(KILL_CHAN1));
		}
	}
}

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

#include "utils/uartstdio.h"

#include "PPM.h"
#include "px4_i2c.h"
#include "servoIn.h"
#include "time_util.h"
#include "px4_kalman.h"
#include "dof.h"
#include "quad_ctrl.h"
#include "messaging.h"

#define SYSCLOCK 80000000
#define RED_LED   GPIO_PIN_1
#define GREEN_LED GPIO_PIN_3
#define BLUE_LED  GPIO_PIN_2

#define GAINS_START_LOC 0x00

#define RC_CHAN1 GPIO_PORTA_BASE,2
#define RC_CHAN2 GPIO_PORTA_BASE,3
#define RC_CHAN3 GPIO_PORTA_BASE,4
#define RC_CHAN4 GPIO_PORTA_BASE,5
#define RC_CHAN5 GPIO_PORTA_BASE,6
#define RC_CHAN6 GPIO_PORTA_BASE,7

#define KILL_CHAN1 GPIO_PORTE_BASE,4
#define KILL_CHAN2 GPIO_PORTE_BASE,5
#define KILL_CHAN3 GPIO_PORTE_BASE,3
#define KILL_CHAN4 GPIO_PORTE_BASE,2
#define KILL_CHAN5 GPIO_PORTE_BASE,1
#define KILL_CHAN6 GPIO_PORTE_BASE,0

enum PID_gains_enum {Kp, Ki, Kd};
bool px4_can_transmit = true;

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
float ms2XY_rate(float ms) {
	return(map(ms, 1.0, 2.0, -1.0, 1.0));
}
float ms2height(float ms) {
	return(map(ms, 1.0, 2.0, 0.5, 1.5));
}
float PID_XY_2ms(float val) {
	return(map(val, -1.0, 1.0, 1.0, 2.0));
}
void ConfigureUART(void);
void sendToSerialPort(kalman_t*, uint16_t);

typedef struct {
	float Kp;
	float Kd;
} Gains_t;
typedef struct {
	Gains_t XY;
	Gains_t Z;
} PID_Gains_t;
typedef union {
	PID_Gains_t PID;
	uint32_t raw[sizeof(PID_Gains_t)];
} PID_Wrapper_t;

typedef struct {
	uint32_t periph;
	uint32_t portBase;
	uint8_t  pinNum		: 4;
	uint8_t  readState	: 1;
	uint8_t  driveState	: 1;
} SwitchData_t;
void initSwitch(uint32_t periph, uint32_t base, uint32_t pin, SwitchData_t *sData) {
	sData->periph = periph;
	sData->portBase = base;
	sData->pinNum = pin;

	SysCtlPeripheralEnable(sData->periph);
	GPIOPinTypeGPIOInput(sData->portBase, sData->pinNum);
	GPIOPadConfigSet(sData->portBase, sData->pinNum, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

	SysCtlDelay(10);	// wait a few clock cycles for the switch signal to settle.

	sData->readState = GPIOPinRead(sData->portBase, sData->pinNum) ? 1 : 0;	// Sample the port with mask
	sData->driveState = sData->readState;
	GPIOPinTypeGPIOOutput(sData->portBase, sData->pinNum);
	uint8_t mask = sData->driveState ? sData->pinNum : 0;
	GPIOPinWrite(sData->portBase, sData->pinNum, mask);
	return;
}
void readSwitch(SwitchData_t *sData) {
	GPIOPinTypeGPIOInput(sData->portBase, sData->pinNum);// Set the GPIO to input
	GPIOPadConfigSet(sData->portBase, sData->pinNum, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);// I may not need this

	SysCtlDelay(5);	// wait a few clock cycles for the switch signal to settle.

	sData->readState = GPIOPinRead(sData->portBase, sData->pinNum) ? 1 : 0;	// Sample the port with mask

	GPIOPinTypeGPIOOutput(sData->portBase, sData->pinNum);
	uint8_t mask = sData->driveState ? sData->pinNum : 0;
	GPIOPinWrite(sData->portBase, sData->pinNum, mask);

	return;
}
void driveSwitch(SwitchData_t *sData, uint8_t direction) {
	sData->driveState = direction;
	uint8_t mask = sData->driveState ? sData->pinNum : 0;
	GPIOPinWrite(sData->portBase, sData->pinNum, mask);

	return;
}
void recordGains(quad_ctrl_t *qc) {
	uint32_t memLoc = GAINS_START_LOC;
	EEPROMProgram((uint32_t*)(qc->xyzh[X_AXIS].rate_gains) , memLoc, sizeof(qc->xyzh[X_AXIS].rate_gains));
	memLoc += sizeof(qc->xyzh[X_AXIS].rate_gains);
	EEPROMProgram((uint32_t *)(qc->xyzh[Z_AXIS].value_gains) , memLoc, sizeof(qc->xyzh[Z_AXIS].value_gains));
	return;
}
void copyGains(quad_ctrl_t *qc) {
	uint32_t memLoc = GAINS_START_LOC;
	EEPROMRead((uint32_t*)(qc->xyzh[X_AXIS].rate_gains) , memLoc, sizeof(qc->xyzh[X_AXIS].rate_gains));
	memLoc += sizeof(qc->xyzh[X_AXIS].rate_gains);
	EEPROMRead((uint32_t*)(qc->xyzh[Z_AXIS].value_gains), memLoc, sizeof(qc->xyzh[Z_AXIS].value_gains));
	return;
}
int main(void) {
	// Set system clock to 80Mhz.
	// Note that SysCtlClockGet() has a bug for this frequency.  Use the above #defined constant "SYSCLOCK" instead.
	SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

	// Init the LEDs on the Launchpad for debugging and init them to off
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);

	SwitchData_t sw[3];
	initSwitch(SYSCTL_PERIPH_GPIOB, GPIO_PORTB_BASE, GPIO_PIN_1, &sw[0]);
	initSwitch(SYSCTL_PERIPH_GPIOB, GPIO_PORTB_BASE, GPIO_PIN_2, &sw[1]);
	initSwitch(SYSCTL_PERIPH_GPIOB, GPIO_PORTB_BASE, GPIO_PIN_3, &sw[2]);


	time_init(SYSCTL_PERIPH_TIMER1, SYSCLOCK, TIMER1_BASE, INT_TIMER1A);	// Chose any open timer

	// TODO Line 66 of this SRC is not abstract
	PPM_init( SYSCTL_PERIPH_TIMER2, SYSCLOCK, TIMER2_BASE, INT_TIMER2A,		// Chose any open timer
			GPIO_PORTB_BASE, GPIO_PIN_6, 4);								// Chose any open port/pin

	servoIn_init(SYSCTL_PERIPH_TIMER4, TIMER4_BASE);		// Chose timer4 until encapsulated
	servoIn_attachPin();									// TODO encapsulate this src

	init_px4_i2c(SYSCTL_PERIPH_I2C3, SYSCTL_PERIPH_GPIOD, SYSCTL_PERIPH_GPIOD,
				SYSCLOCK, I2C3_BASE, GPIO_PORTD_BASE, GPIO_PORTD_BASE,
				GPIO_PIN_0, GPIO_PIN_1, GPIO_PD0_I2C3SCL, GPIO_PD1_I2C3SDA);

	// Turn on Floating point hardware
	FPULazyStackingEnable();
	FPUEnable();

	// Set up UART comms to computer terminal
	ConfigureUART();


	SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
	EEPROMInit();

//    kalman_t *filter = kalman_create();
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

	// Initialize messaging system
	messaging_t messages;
	messaging_init(&messages);
	// Load messages struct with initial values.
	// gains
	messages._KPX = qc.xyzh[0].value_gains[0];
	messages._KIX = qc.xyzh[0].value_gains[1];
	messages._KDX = qc.xyzh[0].value_gains[2];
	messages._KPXdot = qc.xyzh[0].rate_gains[0];
	messages._KIXdot = qc.xyzh[0].rate_gains[1];
	messages._KDXdot = qc.xyzh[0].rate_gains[2];
	messages._KPY = qc.xyzh[1].value_gains[0];
	messages._KIY = qc.xyzh[1].value_gains[1];
	messages._KDY = qc.xyzh[1].value_gains[2];
	messages._KPYdot = qc.xyzh[1].rate_gains[0];
	messages._KIYdot = qc.xyzh[1].rate_gains[1];
	messages._KDYdot = qc.xyzh[1].rate_gains[2];
	messages._KPZ = qc.xyzh[2].value_gains[0];
	messages._KIZ = qc.xyzh[2].value_gains[1];
	messages._KDZ = qc.xyzh[2].value_gains[2];
	messages._KPZdot = qc.xyzh[2].rate_gains[0];
	messages._KIZdot = qc.xyzh[2].rate_gains[1];
	messages._KDZdot = qc.xyzh[2].rate_gains[2];
	messages._KPH = qc.xyzh[3].value_gains[0];
	messages._KIH = qc.xyzh[3].value_gains[1];
	messages._KDH = qc.xyzh[3].value_gains[2];
	// setpoints
	messages._x = qc.xyzh[0].setpt[0];
	messages._y = qc.xyzh[1].setpt[0];
	messages._z = qc.xyzh[2].setpt[0];
	messages._h = qc.xyzh[3].setpt[0];
	messages._xdot = qc.xyzh[0].setpt[1];
	messages._ydot = qc.xyzh[1].setpt[1];
	messages._zdot = qc.xyzh[2].setpt[1];



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
	uint8_t  mode = 3;	// defalut to RC mode

	SysCtlDelay(SYSCLOCK);	// about 3 seconds.  Required for DJI startup

	for(;;) {
		loopTime = millis();

		if(loopTime-switchUpdateTime > 10) {
			switchUpdateTime = loopTime;
			int i;
			for(i=0;i<3;i++)
				readSwitch(&sw[i]);
//			for(i=0;i<3;i++)
//			driveSwitch(&sw[0], sw[0].readState);
//			driveSwitch(&sw[1], sw[1].readState);
//			driveSwitch(&sw[2], sw[2].readState);

		}
		if(loopTime-modeCheckTime > 100) {
			modeCheckTime = loopTime;
			mode = automomousMode(servoIn_getPulse(RC_CHAN5)) ? 3 : pulseUpperThird(servoIn_getPulse(KILL_CHAN5)) ? 1 : 2;
			switch(mode) {
			case(1):	GPIOPinWrite(GPIO_PORTF_BASE, RED_LED | GREEN_LED | BLUE_LED, RED_LED);		break;
			case(2):	GPIOPinWrite(GPIO_PORTF_BASE, RED_LED | GREEN_LED | BLUE_LED, BLUE_LED);	break;
			case(3):	GPIOPinWrite(GPIO_PORTF_BASE, RED_LED | GREEN_LED | BLUE_LED, GREEN_LED);	break;
			}
		}
		// Gains will be sent via Tuning program so we should log gains Atom side
//		if(loopTime-writeEepromTime > 1000) {	// Every now and then, log the gains to EEPROM
//			writeEepromTime = loopTime;
//			recordGains(&qc);
////			EEPROMProgram(gains.raw, 0x0, sizeof(gains.raw));
//		}
		// Gains will be sent via Tuning program so this isn't needed
//		if(loopTime-gainCheckTime > 100) {
//			gainCheckTime = loopTime;
//
////			char buffer[100];
////			uint32_t len = snprintf(buffer, 1000,
////					"%f,\t%f,\t%f,\t%f\n",
////					xy_rateGains[Kp],
////					xy_rateGains[Kd],
////					z_valueGains[Kp],
////					z_valueGains[Kd]);
////			UARTwrite(buffer, len);
//
//			if(mode == 1) {
//				if(      pulseUpperThird(servoIn_getPulse(KILL_CHAN1))) xy_rateGains[Kd] *= 1.01;
//				else if( pulseLowerThird(servoIn_getPulse(KILL_CHAN1))) xy_rateGains[Kd] *= 0.99;
//				if(      pulseUpperThird(servoIn_getPulse(KILL_CHAN2))) xy_rateGains[Kp] *= 1.01;
//				else if( pulseLowerThird(servoIn_getPulse(KILL_CHAN2))) xy_rateGains[Kp] *= 0.99;
//				dof_set_gains(&(qc.xyzh[X_AXIS]), xy_valueGains, xy_rateGains);
//				dof_set_gains(&(qc.xyzh[Y_AXIS]), xy_valueGains, xy_rateGains);
//
//			} else if(mode == 2) {
//				if(      pulseUpperThird(servoIn_getPulse(KILL_CHAN1))) z_valueGains[Kd] *= 1.01;
//				else if( pulseLowerThird(servoIn_getPulse(KILL_CHAN1))) z_valueGains[Kd] *= 0.99;
//				if(      pulseUpperThird(servoIn_getPulse(KILL_CHAN2))) z_valueGains[Kp] *= 1.01;
//				else if( pulseLowerThird(servoIn_getPulse(KILL_CHAN2))) z_valueGains[Kp] *= 0.99;
//				dof_set_gains(&(qc.xyzh[Z_AXIS]), z_valueGains, z_rateGains);
//			}
//		}
		// When the flag for new set point is raised, set new setpoints
		if(messages._cmd & MESSAGING_FLAG_SET_PIDCONST)
		{
			// Clear the flag
			messages._cmd &= ~MESSAGING_FLAG_SET_PIDCONST;
			// Arrays to use dof_set_gains with
			float newGains[3];
			float newRateGains[3];
			newGains[0] = messages._KPX;
			newGains[1] = messages._KIX;
			newGains[2] = messages._KDX;
			newRateGains[0] = messages._KPXdot;
			newRateGains[1] = messages._KIXdot;
			newRateGains[2] = messages._KDXdot;
			dof_set_gains(&qc.xyzh[0], newGains, newRateGains);
			newGains[0] = messages._KPY;
			newGains[1] = messages._KIY;
			newGains[2] = messages._KDY;
			newRateGains[0] = messages._KPYdot;
			newRateGains[1] = messages._KIYdot;
			newRateGains[2] = messages._KDYdot;
			dof_set_gains(&qc.xyzh[1], newGains, newRateGains);
			newGains[0] = messages._KPZ;
			newGains[1] = messages._KIZ;
			newGains[2] = messages._KDZ;
			newRateGains[0] = messages._KPZdot;
			newRateGains[1] = messages._KIZdot;
			newRateGains[2] = messages._KDZdot;
			dof_set_gains(&qc.xyzh[2], newGains, newRateGains);
			newGains[0] = messages._KPH;
			newGains[1] = messages._KIH;
			newGains[2] = messages._KDH;
			newRateGains[0] = 0;
			newRateGains[1] = 0;
			newRateGains[2] = 0;
			dof_set_gains(&qc.xyzh[3], newGains, newRateGains);
		}

		if(loopTime-update_setPoints_time > 20) {
			update_setPoints_time = loopTime;
			// Only get setpoints from rc if rc mode
			if (qc.ctrl_mode == RC_CTRL)
			{
				setpoints[5] = ms2XY_rate(pulse2ms(servoIn_getPulse(RC_CHAN1)));// Y Rate
				setpoints[4] = ms2XY_rate(pulse2ms(servoIn_getPulse(RC_CHAN2)));// X Rate
				setpoints[2] = ms2height(pulse2ms(servoIn_getPulse(RC_CHAN3)));	// Z Absolute
				//setpoints[7] = pulse2ms(servoIn_getPulse(RC_CHAN4));	// Yaw Rate	TODO Add this back in later
				setpoints[0] = setpoints[1] = setpoints[3] = setpoints[6] =
						setpoints[7] = 0;
			}
//			setpoints[2] = ms2height(1.5);
			if(qc.xyzh[Z_AXIS].Uval > 0)	 	driveSwitch(&sw[0], 1);
			else 								driveSwitch(&sw[0], 0);
			if(qc.xyzh[Z_AXIS].setpt[0] >1.0)	driveSwitch(&sw[2], 1);
			else								driveSwitch(&sw[2], 0);

			qc_setSetpt(&qc, setpoints, timestamp_now());
		}

		// New setpoint from Atom
		if(messages._cmd & MESSAGING_FLAG_NEW_SETPOINT)
		{
			// clear flag
			messages._cmd &= ~MESSAGING_FLAG_NEW_SETPOINT;

			// update setpoint array [x, y, z, yaw, x_dot, y_dot, z_dot, yaw_dot]
			setpoints[0] = messages._x;
			setpoints[1] = messages._y;
			setpoints[2] = messages._z;
			setpoints[3] = messages._h;
			setpoints[4] = setpoints[5] = setpoints[6] = setpoints[7] = 0;

			// navigation wants to send relative position for xyz setpoint, so reset state
			// TODO: Make sure navigation still likes relative position for xyz and absolute h
			qc.xyzh[0].state[0] = qc.xyzh[1].state[0] = qc.xyzh[2].state[0] = 0;

			// Send to quadcontrol
			qc_setSetpt(&qc, setpoints, timestamp_now());
		}

		// New dot setpoint from Atom
		if (messages._cmd & MESSAGING_FLAG_SET_DOT_SETPOINT)
		{
			// clear flag
			messages._cmd &= ~MESSAGING_FLAG_SET_DOT_SETPOINT;

			// update setpoint array [x, y, z, yaw, x_dot, y_dot, z_dot, yaw_dot]
			setpoints[4] = messages._xdot;
			setpoints[5] = messages._ydot;
			setpoints[6] = messages._zdot;

			// Send to quadcontrol
			qc_setSetpt(&qc, setpoints, timestamp_now());
		}

		// Atom says land
		if (messages._cmd & MESSAGING_FLAG_LAND)
		{
			messages._cmd &= ~MESSAGING_FLAG_LAND;
			//TODO: Implement landing
		}

		// Atom says takeoff
		if (messages._cmd & MESSAGING_FLAG_TAKEOFF)
		{
			messages._cmd &= ~MESSAGING_FLAG_TAKEOFF;
			//TODO: Implement takeoff
		}

		// Atom wants to give current location
		if (messages._cmd & MESSAGING_FLAG_SET_LOCATION)
		{
			messages._cmd &= ~MESSAGING_FLAG_SET_LOCATION;

		}

		if(loopTime-update_PX4_time > 10 && px4_can_transmit == true) {
			update_PX4_time = loopTime;
			initiate_PX4_transmit();
			px4_can_transmit = false;
//			driveSwitch(&sw[1], 1);
		}
		if(loopTime-test_PX4_time > 25000) {
			test_PX4_time = loopTime;
            uint16_t frameCount = px4_i2c_get_frame_count();
			if(	(loopTime - lastFreshDataTime > 50000) || (frameCount == 65535) || (frameCount == 0) ) {
				driveSwitch(&sw[1], 1);
				UARTprintf("\n\nI2C_fail\n\n");

				SysCtlPeripheralReset(SYSCTL_PERIPH_I2C3);

				SysCtlDelay(100);	// wait a few clock cycles for the switch signal to settle.

				init_px4_i2c(SYSCTL_PERIPH_I2C3, SYSCTL_PERIPH_GPIOD, SYSCTL_PERIPH_GPIOD,
							SYSCLOCK, I2C3_BASE, GPIO_PORTD_BASE, GPIO_PORTD_BASE,
							GPIO_PIN_0, GPIO_PIN_1, GPIO_PD0_I2C3SCL, GPIO_PD1_I2C3SDA);


			}
            else driveSwitch(&sw[1], 0);

//			char buffer[100];
//			uint32_t len = snprintf(buffer, 100,
//					"%d\n",
//					px4_i2c_get_frame_count());
//			UARTwrite(buffer, len);
		}
        if(px4_i2c_dataFresh()) {
        	lastFreshDataTime = loopTime;
            uint16_t frameCount = px4_i2c_get_frame_count();
            if(frameCount != oldFrameCount) {


                kalman_process_data(filter,
                                    px4_i2c_get_flow_comp_m_x(),
                                    px4_i2c_get_flow_comp_m_y(),
                                    px4_i2c_getHeight(),
                                    px4_i2c_get_gyro_z_rate(),
                                    px4_i2c_get_gyro_range(),
                                    px4_i2c_get_qual(),
                                    px4_i2c_getTimestep(),
                                    timestamp_now());
                if((frameCount != 65535) && (frameCount != 0))
                	sendToSerialPort(filter, frameCount);

            }
            oldFrameCount = frameCount;
            px4_i2c_makeDataStale();
            px4_can_transmit = true;
        }
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

		if(loopTime - update_DJI_time > 10) {
			update_DJI_time = loopTime;
			if(mode == 1 || mode == 2){		// autonomous mode.  do something smart
				PPM_setPulse(0, servoIn_getPulse(RC_CHAN1));
				PPM_setPulse(1, servoIn_getPulse(RC_CHAN2));
				float zPulse = PID_XY_2ms(qc.xyzh[Z_AXIS].Uval);
				zPulse = zPulse > 1.2 ? zPulse : 1.2;
				PPM_setPulse(2, ms2pulse(zPulse));	// Z control to DJI
				PPM_setPulse(3, servoIn_getPulse(RC_CHAN4));



			} else {  		// RC passthrough.  Dump RC Data directly into the DJI
				PPM_setPulse(0, servoIn_getPulse(RC_CHAN1));	// Y Accel
				PPM_setPulse(1, servoIn_getPulse(RC_CHAN2));	// X Accel
				PPM_setPulse(2, servoIn_getPulse(RC_CHAN3));	// Z Accel
				PPM_setPulse(3, servoIn_getPulse(RC_CHAN4));	// Yaw Rate

			}
		}
	}
}
void sendToSerialPort(kalman_t* filter, uint16_t frameCount) {
	char buffer[150];
	uint32_t len = snprintf(buffer, 150,
			"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
			frameCount,
			px4_i2c_get_pixel_flow_x_sum(),
			px4_i2c_get_pixel_flow_y_sum(),
			px4_i2c_get_flow_comp_m_x(),
			px4_i2c_get_flow_comp_m_y(),
			px4_i2c_get_qual(),
			px4_i2c_get_gyro_x_rate(),
			px4_i2c_get_gyro_y_rate(),
			px4_i2c_get_gyro_z_rate(),
			px4_i2c_get_gyro_range(),
			px4_i2c_getTimestep(),
			px4_i2c_getHeight(),
			filter->xdot,
			filter->ydot,
			filter->z,
			filter->zdot,
			filter->P11,
			filter->P22,
			filter->P33,
			filter->P34,
			filter->P44);
	UARTwrite(buffer, len);
	return;
}

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void ConfigureUART(void) {
	//
	// Enable the GPIO Peripheral used by the UART.
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	//
	// Enable UART0
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

	//
	// Configure GPIO Pins for UART mode.
	//
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	//
	// Use the internal 16MHz oscillator as the UART clock source.
	//
	UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

	//
	// Initialize the UART for console I/O.
	//
	UARTStdioConfig(0, 115200, 16000000);

}

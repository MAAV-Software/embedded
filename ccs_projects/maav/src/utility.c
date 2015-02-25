/*
 * utility.c
 *
 * Implementation of general utility library for MAAV controls.
 *
 *  Created on: Dec 18, 2014
 *      Author: Sajan Patel, Jonathan Kurzer
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

#include "utils/uartstdio.h"

#include "px4_kalman.h"
#include "PPM.h"
#include "dof.h"
#include "quad_ctrl.h"
#include "px4_i2c.h"
#include "utility.h"
#include "messaging/data_link.h"
#include "time_util.h"

/***************** Utility Functions for RC Controller ************************/
// Returns true if the pulse is longer than 1.66ms
bool pulseUpperThird(volatile uint32_t pulseWidth)
{
	return (pulseWidth > (SYSCLOCK / 602)) ? true : false;
}

// Returns true if the pulse is shorter than 1.33ms
bool pulseLowerThird(volatile uint32_t pulseWidth)
{
	return (pulseWidth < (SYSCLOCK / 752)) ? true : false;
}

// Returns true if the quadrotor is in autonomous mode
bool automomousMode(volatile uint32_t pulseWidth)
{
	return pulseLowerThird(pulseWidth);
}

// Converts pulse ticks to miliseconds
float pulse2ms(uint32_t pulse)
{
	static float one_ms = SYSCLOCK / 1000;
	return ((float)(pulse)) / one_ms;
}

// Converts miliseconds to pulse ticks
uint32_t ms2pulse(float ms)
{
	static float one_ms = SYSCLOCK / 1000;
	return (uint32_t)(ms * one_ms);
}

// Maps x which is in the range of [fromLow, fromHigh] into the range of
// [toLow, toHigh] and returns the result.
float map(float x, float fromLow, float fromHigh, float toLow, float toHigh)
{
	return ((x - fromLow) * (toHigh - toLow)) / ((fromHigh - fromLow) + toLow);
}

// Calculate XY_rate pass-through from pulse width
// (i.e. signal from RC controller modified and passed to DJI)
float ms2XY_rate(float ms)
{
	return(map(ms, 1.0, 2.0, -1.0, 1.0));
}

// Calculates height pass-through from pulse width
float ms2height(float ms)
{
	return(map(ms, 1.0, 2.0, 0.5, 1.5));
}

// Calcualtes PID XY setpoints from pulse width
float PID_XY_2ms(float val)
{
	return(map(val, -1.0, 1.0, 1.0, 2.0));
}

/**************** Utility Functions for Lighted 3-pos Switch ******************/
// Initilializes the 3-position, lighted switch
void initSwitch(uint32_t periph, uint32_t base, uint32_t pin,
				SwitchData_t *sData)
{
	sData->periph = periph;
	sData->portBase = base;
	sData->pinNum = pin;

	SysCtlPeripheralEnable(sData->periph);
	GPIOPinTypeGPIOInput(sData->portBase, sData->pinNum);
	GPIOPadConfigSet(sData->portBase, sData->pinNum, GPIO_STRENGTH_2MA,
					 GPIO_PIN_TYPE_STD_WPU);

	SysCtlDelay(10); // wait a few clock cycles for the switch signal to settle.

	// Sample the port with mask
	sData->readState = GPIOPinRead(sData->portBase, sData->pinNum) ? 1 : 0;
	sData->driveState = sData->readState;
	GPIOPinTypeGPIOOutput(sData->portBase, sData->pinNum);

	uint8_t mask = sData->driveState ? sData->pinNum : 0;
	GPIOPinWrite(sData->portBase, sData->pinNum, mask);

	return;
}

// Reads the switch position
void readSwitch(SwitchData_t *sData)
{
	GPIOPinTypeGPIOInput(sData->portBase, sData->pinNum);// Set GPIO to input
	GPIOPadConfigSet(sData->portBase, sData->pinNum, GPIO_STRENGTH_2MA,
					 GPIO_PIN_TYPE_STD_WPU);// I may not need this

	SysCtlDelay(5);	// wait a few clock cycles for the switch signal to settle.

	// Sample the port with mask
	sData->readState = GPIOPinRead(sData->portBase, sData->pinNum) ? 1 : 0;
	GPIOPinTypeGPIOOutput(sData->portBase, sData->pinNum);

	uint8_t mask = sData->driveState ? sData->pinNum : 0;
	GPIOPinWrite(sData->portBase, sData->pinNum, mask);

	return;
}

// Drives the switch's LED
void driveSwitch(SwitchData_t *sData, uint8_t direction)
{
	sData->driveState = direction;
	uint8_t mask = sData->driveState ? sData->pinNum : 0;
	GPIOPinWrite(sData->portBase, sData->pinNum, mask);

	return;
}

/****************** Serial Port/Kalman Debug Utility Functions ****************/
void sendToSerialPort(kalman_t* filter, uint16_t frameCount)
{
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

// Configure the UART and its pins.  This must be called before UARTprintf().
void ConfigureUART(void)
{
	// Enable the GPIO Peripheral used by the UART.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	// Enable UART0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

	// Configure GPIO Pins for UART mode.
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// Use the internal 16MHz oscillator as the UART clock source.
	UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

	// Initialize the UART for console I/O.
	UARTStdioConfig(0, 115200, 16000000);
}

/*********************** PID Gains Memory Utilities ***************************/
// Record gains from quad_ctrl into EEPROM
void recordGains(quad_ctrl_t *qc)
{
	uint32_t memLoc = GAINS_START_LOC;
	EEPROMProgram((uint32_t*)(qc->xyzh[X_AXIS].rate_gains) , memLoc,
				  sizeof(qc->xyzh[X_AXIS].rate_gains));
	memLoc += sizeof(qc->xyzh[X_AXIS].rate_gains);
	EEPROMProgram((uint32_t *)(qc->xyzh[Z_AXIS].value_gains) , memLoc,
				  sizeof(qc->xyzh[Z_AXIS].value_gains));
	return;
}

// Copy gains from EEPROM into quad_ctrl
void copyGains(quad_ctrl_t *qc)
{
	uint32_t memLoc = GAINS_START_LOC;
	EEPROMRead((uint32_t*)(qc->xyzh[X_AXIS].rate_gains) , memLoc,
			   sizeof(qc->xyzh[X_AXIS].rate_gains));
	memLoc += sizeof(qc->xyzh[X_AXIS].rate_gains);
	EEPROMRead((uint32_t*)(qc->xyzh[Z_AXIS].value_gains), memLoc,
			   sizeof(qc->xyzh[Z_AXIS].value_gains));
	return;
}

/********************** Message Handling Utilities ****************************/

void tuningMessageQuadCtrlChangesHandler(quad_ctrl_t *qc, tuning_t *message)
{
	// Handling of PID gain changes
	if (message->cmd & CMD_TUNING_SETPID_X)
	{
		// Mark as handled
		message->cmd &= ~CMD_TUNING_SETPID_X;
		/*
		 * Set the gains. This relies on the fact that the struct is arranged so that
		 * KP is immediately followed by KI then KD for all gains so it should be possible
		 * to take addr of first one, the cast to float array and all should work, assuming
		 * the compiler doesn't do something weird with the alignment on us
		 */
		dof_set_gains(&qc->xyzh[X_AXIS], (float *)(&(message->KPX)), qc->xyzh[X_AXIS].rate_gains);
	}
	if (message->cmd & CMD_TUNING_SETPID_Y)
	{
		// Mark as handled
		message->cmd &= ~CMD_TUNING_SETPID_Y;
		/*
		 * Set the gains. This relies on the fact that the struct is arranged so that
		 * KP is immediately followed by KI then KD for all gains so it should be possible
		 * to take addr of first one, the cast to float array and all should work, assuming
		 * the compiler doesn't do something weird with the alignment on us
		 */
		dof_set_gains(&qc->xyzh[Y_AXIS], (float *)(&(message->KPY)), qc->xyzh[Y_AXIS].rate_gains);
	}
	if (message->cmd & CMD_TUNING_SETPID_Z)
	{
		// Mark as handled
		message->cmd &= ~CMD_TUNING_SETPID_Z;
		/*
		 * Set the gains. This relies on the fact that the struct is arranged so that
		 * KP is immediately followed by KI then KD for all gains so it should be possible
		 * to take addr of first one, the cast to float array and all should work, assuming
		 * the compiler doesn't do something weird with the alignment on us
		 */
		dof_set_gains(&qc->xyzh[Z_AXIS], (float *)(&(message->KPZ)), qc->xyzh[Z_AXIS].rate_gains);
	}
	if (message->cmd & CMD_TUNING_SETPID_H)
	{
		// Mark as handled
		message->cmd &= ~CMD_TUNING_SETPID_H;
		/*
		 * Set the gains. This relies on the fact that the struct is arranged so that
		 * KP is immediately followed by KI then KD for all gains so it should be possible
		 * to take addr of first one, the cast to float array and all should work, assuming
		 * the compiler doesn't do something weird with the alignment on us
		 */
		dof_set_gains(&qc->xyzh[YAW], (float *)(&(message->KPH)), qc->xyzh[YAW].rate_gains);
	}
	if (message->cmd & CMD_TUNING_SETPID_XDOT)
	{
		// Mark as handled
		message->cmd &= ~CMD_TUNING_SETPID_XDOT;
		/*
		 * Set the gains. This relies on the fact that the struct is arranged so that
		 * KP is immediately followed by KI then KD for all gains so it should be possible
		 * to take addr of first one, the cast to float array and all should work, assuming
		 * the compiler doesn't do something weird with the alignment on us
		 */
		dof_set_gains(&qc->xyzh[X_AXIS], qc->xyzh[X_AXIS].value_gains, (float *)(&(message->KPXdot)));
	}
	if (message->cmd & CMD_TUNING_SETPID_YDOT)
	{
		// Mark as handled
		message->cmd &= ~CMD_TUNING_SETPID_YDOT;
		/*
		 * Set the gains. This relies on the fact that the struct is arranged so that
		 * KP is immediately followed by KI then KD for all gains so it should be possible
		 * to take addr of first one, the cast to float array and all should work, assuming
		 * the compiler doesn't do something weird with the alignment on us
		 */
		dof_set_gains(&(qc->xyzh[Y_AXIS]), qc->xyzh[Y_AXIS].value_gains, (float *)(&(message->KPYdot)));
	}
	if (message->cmd & CMD_TUNING_SETPID_ZDOT)
	{
		// Mark as handled
		message->cmd &= ~CMD_TUNING_SETPID_ZDOT;
		/*
		 * Set the gains. This relies on the fact that the struct is arranged so that
		 * KP is immediately followed by KI then KD for all gains so it should be possible
		 * to take addr of first one, the cast to float array and all should work, assuming
		 * the compiler doesn't do something weird with the alignment on us
		 */
		dof_set_gains(&(qc->xyzh[Z_AXIS]), qc->xyzh[Z_AXIS].value_gains, (float *)(&(message->KPZdot)));
	}

	/* Uncomment this block and comment the above block if compiler alignment
	 * of structs makes the above way with arrays and stuff fail
	 */
	/*
	if (message->cmd & CMD_TUNING_SETPID_X)
	{
		message->cmd &= ~CMD_TUNING_SETPID_X; // Mark as handled
		float new_gains[3];
		new_gains[0] = message->KPX;
		new_gains[1] = message->KIX;
		new_gains[2] = message->KDX;
		dof_set_gains(&qc->xyzh[X_AXIS], new_gains, qc->xyzh[X_AXIS].rate_gains);
	}

	if (message->cmd & CMD_TUNING_SETPID_Y)
	{
		message->cmd &= ~CMD_TUNING_SETPID_Y; // Mark as handled
		float new_gains[3];
		new_gains[0] = message->KPY;
		new_gains[1] = message->KIY;
		new_gains[2] = message->KDY;
		dof_set_gains(&qc->xyzh[Y_AXIS], new_gains, qc->xyzh[Y_AXIS].rate_gains);
	}

	if (message->cmd & CMD_TUNING_SETPID_Z)
	{
		message->cmd &= ~CMD_TUNING_SETPID_Z; // Mark as handled
		float new_gains[3];
		new_gains[0] = message->KPZ;
		new_gains[1] = message->KIZ;
		new_gains[2] = message->KDZ;
		dof_set_gains(&qc->xyzh[Z_AXIS], new_gains, qc->xyzh[Z_AXIS].rate_gains);
	}

	if (message->cmd & CMD_TUNING_SETPID_H)
	{
		message->cmd &= ~CMD_TUNING_SETPID_H; // Mark as handled
		float new_gains[3];
		new_gains[0] = message->KPH;
		new_gains[1] = message->KIH;
		new_gains[2] = message->KDH;
		dof_set_gains(&qc->xyzh[YAW], new_gains, qc->xyzh[YAW].rate_gains);
	}

	if (message->cmd & CMD_TUNING_SETPID_XDOT)
	{
		message->cmd &= ~CMD_TUNING_SETPID_XDOT; // Mark as handled
		float new_gains[3];
		new_gains[0] = message->KPXdot;
		new_gains[1] = message->KIXdot;
		new_gains[2] = message->KDXdot;
		dof_set_gains(&qc->xyzh[X_AXIS], qc->xyzh[X_AXIS].value_gains, new_gains);
	}

	if (message->cmd & CMD_TUNING_SETPID_YDOT)
	{
		message->cmd &= ~CMD_TUNING_SETPID_YDOT; // Mark as handled
		float new_gains[3];
		new_gains[0] = message->KPYdot;
		new_gains[1] = message->KIYdot;
		new_gains[2] = message->KDYdot;
		dof_set_gains(&qc->xyzh[Y_AXIS], qc->xyzh[Y_AXIS].value_gains, new_gains);
	}

	if (message->cmd & CMD_TUNING_SETPID_ZDOT)
	{
		message->cmd &= ~CMD_TUNING_SETPID_ZDOT; // Mark as handled
		float new_gains[3];
		new_gains[0] = message->KPZdot;
		new_gains[1] = message->KIZdot;
		new_gains[2] = message->KDZdot;
		dof_set_gains(&qc->xyzh[Z_AXIS], qc->xyzh[Z_AXIS].value_gains, new_gains);
	}
	*/

	// Handling of Setpoint changes
	// Fetch old setpts into setpt array [x, y, z, yaw, x_dot, y_dot, z_dot, yaw_dot]
	float setpts[8];
	setpts[0] = qc->xyzh[X_AXIS].setpt[0];
	setpts[1] = qc->xyzh[Y_AXIS].setpt[0];
	setpts[2] = qc->xyzh[Z_AXIS].setpt[0];
	setpts[3] = qc->xyzh[YAW].setpt[0];
	setpts[4] = qc->xyzh[X_AXIS].setpt[1];
	setpts[5] = qc->xyzh[Y_AXIS].setpt[1];
	setpts[6] = qc->xyzh[Z_AXIS].setpt[1];
	setpts[7] = qc->xyzh[YAW].setpt[1];

	// Make the requested changes to the setpoint array
	if (message->cmd & CMD_TUNING_SETPOINT_X)
	{
		message->cmd &= ~CMD_TUNING_SETPOINT_X;
		setpts[0] = message->x;
	}

	if (message->cmd & CMD_TUNING_SETPOINT_Y)
	{
		message->cmd &= ~CMD_TUNING_SETPOINT_Y;
		setpts[1] = message->y;
	}

	if (message->cmd & CMD_TUNING_SETPOINT_Z)
	{
		message->cmd &= ~CMD_TUNING_SETPOINT_Z;
		setpts[2] = message->z;
	}

	if (message->cmd & CMD_TUNING_SETPOINT_H)
	{
		message->cmd &= ~CMD_TUNING_SETPOINT_H;
		setpts[3] = message->h;
	}

	if (message->cmd & CMD_TUNING_DOTSETPOINT_X)
	{
		message->cmd &= ~CMD_TUNING_DOTSETPOINT_X;
		setpts[4] = message->xdot;
	}

	if (message->cmd & CMD_TUNING_DOTSETPOINT_Y)
	{
		message->cmd &= ~CMD_TUNING_DOTSETPOINT_Y;
		setpts[5] = message->ydot;
	}

	if (message->cmd & CMD_TUNING_DOTSETPOINT_Z)
	{
		message->cmd &= ~CMD_TUNING_DOTSETPOINT_Z;
		setpts[6] = message->zdot;
	}

	// Set the setpoints
	qc_setSetpt(qc, setpts, timestamp_now());

}

void targetMessageQuadCtrlChangesHandler(quad_ctrl_t *qc, target_t *message)
{
	// Handling of Setpoint changes
	// Fetch old setpts into setpt array [x, y, z, yaw, x_dot, y_dot, z_dot, yaw_dot]
	float setpts[8];
	setpts[0] = qc->xyzh[X_AXIS].setpt[0];
	setpts[1] = qc->xyzh[Y_AXIS].setpt[0];
	setpts[2] = qc->xyzh[Z_AXIS].setpt[0];
	setpts[3] = qc->xyzh[YAW].setpt[0];
	setpts[4] = 0;
	setpts[5] = 0;
	setpts[6] = 0;
	setpts[7] = 0;

	// Make the requested changes to the setpoint array
	if (message->cmd & CMD_TARGET_SETPOINT)
	{
		message->cmd &= ~CMD_TARGET_SETPOINT;
		setpts[0] = message->x;
		setpts[1] = message->y;
		setpts[2] = message->z;
		setpts[3] = message->h;
	}

	// Set the setpoints
	qc_setSetpt(qc, setpts, timestamp_now());

}

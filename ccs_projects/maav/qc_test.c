/*
 * qc_test.c
 *
 *  Created on: Dec 21, 2014
 *      Author: Sajan Patel
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

#include "PPM.h"
#include "px4_i2c.h"
#include "servoIn.h"
#include "time_util.h"
#include "px4_kalman.h"
#include "dof.h"
#include "quad_ctrl.h"
#include "utility.h"

#define MAXSIZE 1000

// Logging buffers
float all_states[MAXSIZE][8];
float all_setpts[MAXSIZE][8];

int main(void)
{
	int i, j;

	// init quad ctrl struct
	quad_ctrl_t qc;
	qc_init(&qc);

	// Initial State/Setpt [x, y, z, yaw, x_dot, y_dot, z_dot, yaw_dot]
	float state[8] = { 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	float setpt[8] = { 10.0, 10.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	float time = 0.0;

	for (i = 0; i < MAXSIZE; ++i)
	{
		time = i;
		qc_setState(&qc, state, time);
		qc_setSetpt(&qc, setpt, time);
		qc_runPID(&qc);

		for (j = 0; j < 8; ++j) // log states/setpts
		{
			all_states[i][j] = state[j];
			all_setpts[i][j] = setpt[j];
		}

		// create new state/setpts here

	}
}


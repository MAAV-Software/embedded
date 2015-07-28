/*
 * I2C_hw.hpp
 *
 *  Created on: Jun 24, 2015
 *      Author: Zhengjie
 */

#ifndef I2C_HW_HPP_
#define I2C_HW_HPP_

#include "I2C_defines.hpp"

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/i2c.h"
#include "driverlib/timer.h"
#include "utils/uartstdio.h"
#include "sensorlib/i2cm_drv.h"


// The I2C master driver instance data.
extern tI2CMInstance g_sI2CMInst;

// A boolean that is set when an I2C transaction is completed.
extern bool g_bI2CMDone;

//***************************State Machine**************************************
// Keep track of state machine, meaning the state just done
typedef enum _state {Lidar_1, Lidar_2, PX4_1 } State;
volatile State nextstate, currentstate;
extern uint32_t LidarTime, PX4Time;

extern uint8_t Command[];

// The interrupt handler for the I2C module.
void I2CMasterIntHandler(void);

// The function that is provided by this Run as a callback when I2C transactions have completed.
void I2CMCallback(void *, uint_fast8_t);

void ConfigUART(void);

void ConfigI2C(void);

void TimerInit(void);

#endif /* I2C_HW_HPP_ */

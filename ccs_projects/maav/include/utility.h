/*
 * utility.h
 *
 * General utility library for MAAV controls.
 *
 *  Created on: Dec 18, 2014
 *      Author: Sajan Patel, Jonathan Kurzer
 */

#ifndef UTILITY_H_
#define UTILITY_H_

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "px4_kalman.h"
#include "quad_ctrl.h"
#include "messaging/target_t.h"
#include "messaging/tuning_t.h"
#include "messaging/position_t.h"
#include "messaging/feedback_t.h"
#include "messaging/data_link.h"

/************************* Utility (Global) Defines ***************************/
// System Clock Frequency
#define SYSCLOCK 80000000

// GPIO Pins for LEDs
#define RED_LED   GPIO_PIN_1
#define GREEN_LED GPIO_PIN_3
#define BLUE_LED  GPIO_PIN_2

// Starting address of Gains in EEPROM
#define GAINS_START_LOC 0x00

// GPIO Port Basses for pilot RC controller
#define RC_CHAN1 GPIO_PORTA_BASE,2
#define RC_CHAN2 GPIO_PORTA_BASE,3
#define RC_CHAN3 GPIO_PORTA_BASE,4
#define RC_CHAN4 GPIO_PORTA_BASE,5
#define RC_CHAN5 GPIO_PORTA_BASE,6
#define RC_CHAN6 GPIO_PORTA_BASE,7

// GPIO Port Basses for Kill Switch (also an RC controller)
#define KILL_CHAN1 GPIO_PORTE_BASE,4
#define KILL_CHAN2 GPIO_PORTE_BASE,5
#define KILL_CHAN3 GPIO_PORTE_BASE,3
#define KILL_CHAN4 GPIO_PORTE_BASE,2
#define KILL_CHAN5 GPIO_PORTE_BASE,1
#define KILL_CHAN6 GPIO_PORTE_BASE,0

/********************** Utility Data Structures *******************************/
// Enum for gain indeces in data structures
enum PID_gains_enum {Kp, Ki, Kd};

// Structs for reading and saving PID Gains
typedef struct
{
	float Kp;
	float Kd;
} Gains_t;

typedef struct
{
	Gains_t XY;
	Gains_t Z;
} PID_Gains_t;

typedef union
{
	PID_Gains_t PID;
	uint32_t raw[sizeof(PID_Gains_t)];
} PID_Wrapper_t;

// Struct for lighted 3-position switch states
typedef struct
{
	uint32_t periph;
	uint32_t portBase;
	uint8_t  pinNum		: 4;
	uint8_t  readState	: 1;
	uint8_t  driveState	: 1;
} SwitchData_t;

/************************* Utility Functions **********************************/
// Returns true if the pulse is longer than 1.66ms
bool pulseUpperThird(volatile uint32_t pulseWidth);

// Returns true if the pulse is shorter than 1.33ms
bool pulseLowerThird(volatile uint32_t pulseWidth);

// Returns true if the quadrotor is in autonomous mode
bool automomousMode(volatile uint32_t pulseWidth);

// Converts pulse ticks to miliseconds
float pulse2ms(uint32_t pulse);

// Converts miliseconds to pulse ticks
uint32_t ms2pulse(float ms);

// Maps x which is in the range of [fromLow, fromHigh] into the range of
// [toLow, toHigh] and returns the result.
float map(float x, float fromLow, float fromHigh, float toLow, float toHigh);

// Calculate XY_rate pass-through from pulse width
// (i.e. signal from RC controller modified and passed to DJI)
float ms2XY_rate(float ms);

// Calculates height pass-through from pulse width
float ms2height(float ms);

// Calcualtes PID XY setpoints from pulse width
float PID_XY_2ms(float val);

// Initilializes the 3-position, lighted switch
void initSwitch(uint32_t periph, uint32_t base, uint32_t pin,
				SwitchData_t *sData);

// Reads the switch position
void readSwitch(SwitchData_t *sData);

// Drives the switch's LED
void driveSwitch(SwitchData_t *sData, uint8_t direction);


/****************** Serial Port/Kalman Debug Utility Functions ****************/
// Configures UART serial port for communication to Atom
void ConfigureUART(void);

// Writes kalman filter data over the serial port to be logged by the Atom
void sendToSerialPort(kalman_t*, uint16_t);


/*********************** PID Gains Memory Utilities ***************************/
// Record gains from quad_ctrl into EEPROM
void recordGains(quad_ctrl_t *qc);

// Copy gains from EEPROM into quad_ctrl
void copyGains(quad_ctrl_t *qc);


/********************** Message Handling Utilities ****************************/
// Does the requested changes to quad control struct based on tuning message
void tuningMessageQuadCtrlChangesHandler(quad_ctrl_t *qc, tuning_t *message);

// Does the requested changes to quad control struct based on target message
void targetMessageQuadCtrlChangesHandler(quad_ctrl_t *qc, target_t *message);

#endif /* UTILITY_H_ */

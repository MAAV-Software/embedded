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

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>



/************************* Utility (Global) Defines ***************************/


// Starting address of Gains in EEPROM
#define GAINS_START_LOC 0x00

/********************** Utility Data Structures *******************************/
// Enum for gain indeces in data structures
//enum PID_gains_enum {Kp, Ki, Kd};

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


/************************* Utility Functions **********************************/


/****************** Serial Port/Kalman Debug Utility Functions ****************/
// Configures UART serial port for communication to Atom
void ConfigureUART(void);

// Writes kalman filter data over the serial port to be logged by the Atom
//void sendToSerialPort(kalman_t*, uint16_t);


/*********************** PID Gains Memory Utilities ***************************/
// Record gains from quad_ctrl into EEPROM
//void recordGains(quad_ctrl_t *qc);

// Copy gains from EEPROM into quad_ctrl
//void copyGains(quad_ctrl_t *qc);


/********************** Message Handling Utilities ****************************/
// Does the requested changes to quad control struct based on tuning message
//void tuningMessageQuadCtrlChangesHandler(quad_ctrl_t *qc, tuning_t *message);

// Does the requested changes to quad control struct based on target message
//void targetMessageQuadCtrlChangesHandler(quad_ctrl_t *qc, target_t *message);

#ifdef __cplusplus
}
#endif

#endif /* UTILITY_H_ */

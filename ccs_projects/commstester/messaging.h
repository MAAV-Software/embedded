/*
 * messaging.h
 *
 *  Created on: Nov 20, 2014
 *      Author: Sasawat
 */

#ifndef MESSAGING_H_
#define MESSAGING_H_

#include "protobuf.h"

/**
 * The UART base in which messaging is to run
 */
#define MESSAGING_UART_BASE UART2_BASE

/**
 * The minimum length a valid message can be (1byte/ea: size, varint 1, cmd, varint 2, timestamp)
 */
#define MESSAGING_MIN_VALID_MESSAGE_SIZE 5

/**
 * The maximum length of the size delimeter
 */
#define MESSAGING_SIZE_DELIMIT_MAX_LEN 3

/*==========Defines for command flags========================================*/

//Atom -> Tiva: Set new setpoint (Target or Tuning Message)
//Also include new f32[XYZH]Desired values (if changed)
#define MESSAGING_FLAG_NEW_SETPOINT 1
//Atom -> Tiva: This is your current location (Pose Message)
//Also include new f32[XYZ]Reported (if changed)
#define MESSAGING_FLAG_SET_LOCATION 2
//Atom -> Tiva: Set new PID Constants (Tuning Message)
//Also include the new PID consts (e.g. KPX, KIZDot, etc.) since the last message.
//If first transmit, all KPID Values must be given for now
#define MESSAGING_FLAG_SET_PIDCONST 4
//Atom -> Tiva: Land (Any Message, (usually target??))
//No other values needed
#define MESSAGING_FLAG_LAND 8
//Atom -> Tiva: Takeoff (Any Message, (usually target??))
//No other values needed
#define MESSAGING_FLAG_TAKEOFF 16
//Atom -> Tiva: Set new dot setpoints (Tuning Message)
//Also include new f32K[PID][XYZH]Dot values (if changed)
#define MESSAGING_FLAG_SET_DOT_SETPOINT 32

/*==========End Defines for command flags====================================*/

/**
 * Initializes messaging with all messages received stored in messagedata
 */
void messaging_init(messaging_t *messagedata);



#endif /* MESSAGING_H_ */

/*
 * DataLink is the communications protocol, combining LCM with a custom Transport Layer,
 * used over the serial link between Navigation and Controls on the quadrotor. '
 * Consider the typical LCM link stack of LCM, UDP, IP, Ethernet.
 * The stack for LCM on the quadrotor would be DataLink/LCM, UART driver, UART.
 *
 *  Created on: May 20, 2015
 *      Author: Clark Zhang, Sasawat Prankprakma
 */

#ifndef DATALINK_HPP_
#define DATALINK_HPP_

#include <stdint.h>

#include "RingBuffer.hpp"
#include "messaging/lcmlite.h"
#include "messaging/emergency_t.h"
#include "messaging/feedback_t.h"
#include "messaging/gains_t.h"
#include "messaging/raw_pose_t.h"
#include "messaging/setpt_t.h"
#include "messaging/MessageHandler.hpp"
#include "messaging/TransmitHandler.hpp"
#include "messaging/Decoder.hpp"

/*
A Data link layer packet will look at like
<start delimiter> - 1 byte
<length of message> - 2 to 4 bytes (depending if they have to be escaped) (big endian)
<msg> - msg with escaped characters
<checksum> - 1 to 2 bytes (depending if it needs to be escaped)

checksum is the sum of all the data bytes (unescaped), take only the least significant byte, and subtract from 0xFF.
NOTE: probably should include the length bytes in the checksum

As our RingBuffer will be 256 for reading in, that means the maximum size of a decoded/pre-encoded message is 120 bytes.
*/
class DataLink
{
public:
	DataLink(void (*f)(const uint8_t*,uint32_t));
	~DataLink() { delete rb; }
	
	void recv(uint8_t datum) { rb->push(datum); }
	void process_recv();
	
	// Msg senders
	void send(emergency_t *msg);
	void send(feedback_t *msg);

	// Getters for received messages
	gains_t getGainsMsg() const { return msgHandler.gains; }
	setpt_t getSetptMsg() const { return msgHandler.setpt; }
	raw_pose_t getRawPoseMsg() const { return msgHandler.rawPose; }

private:
	lcmlite_t lcm; // lcmlite struct

	// subscription types for the messages received
	lcmlite_subscription_t gainsSub;	
	lcmlite_subscription_t setptSub;
	lcmlite_subscription_t rawPoseSub;

	// Lcm Message Handler for recieved messages
	MessageHandler msgHandler;

	// Handler for sending lcm messages
	TransmitHandler msgSender;
	
	uint8_t rbBack[256]; // data allocated for ringbuffer	
	RingBuffer<256> *rb; // ringbuffer for incoming bytes
	
	// Decoder for Transport Layer
	Decoder d;
};


#endif /* DATALINK_HPP_ */

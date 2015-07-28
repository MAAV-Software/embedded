/*
 * DataLink is the Transport Network Layer Protocol used over the serial
 * link between Navigation and Controls on the quadrotor. Consider the typical
 * LCM link stack of LCM, UDP, IP, Ethernet. The stack for LCM on the quadrotor
 * would be LCM, DataLink, UART.
 *
 *  Created on: May 20, 2015
 *      Author: Clark Zhang, Sasawat Prankprakma
 */

#include <cstring>
#include <stdint.h>
#include "messaging/lcmlite.h"
#include "messaging/DataLinkDefines.hpp"
#include "messaging/RingBuffer.hpp"
#include "messaging/DataLink.hpp"
#include "messaging/Decoder.hpp"
#include "messaging/MessageHandler.hpp"
#include "messaging/TransmitHandler.hpp"

//LCM Sender Address (we don't use UDP, it just has to be something)
static const uint64_t ATOM_ADDR = 9001;

using namespace std;

DataLink::DataLink(void (*f)(const uint8_t*, uint32_t))
{
	// initialize Ringbuffer
	rb = new RingBuffer<256>(rbBack);

    //Initialize LCM and handlers
	msgSender = TransmitHandler(f);
    lcmlite_init(&lcm, transmitPacket, &msgSender);

    //Initialize LCM subscriptions
    gainsSub.callback = callback;
    gainsSub.channel = "GNS";
    gainsSub.user = &msgHandler;
    rawPoseSub.callback = callback;
    rawPoseSub.channel = "RWP";
    rawPoseSub.user = &msgHandler;
    setptSub.callback = callback;
    setptSub.channel = "SET";
    setptSub.user = &msgHandler;

    //Subscribe LCM subscriptions
    lcmlite_subscribe(&lcm, &gainsSub);
    lcmlite_subscribe(&lcm, &rawPoseSub);
    lcmlite_subscribe(&lcm, &setptSub);
}

void DataLink::send(emergency_t *msg)
{
	lcmlite_publish(&lcm, "EMS", msg, sizeof(emergency_t));	
}

void DataLink::send(feedback_t *msg)
{
	lcmlite_publish(&lcm, "FEB", msg, sizeof(feedback_t));
}

void DataLink::process_recv()
{
    while(rb->unread())
    {
        d.push(rb->pop()); // fetch and parse the next byte in the ringbuffer

        if(d.isError()) d.reset(); // check for error
        
		if(d.isDone()) // check for done
        {
            lcmlite_receive_packet(&lcm, d.packetData(), d.packetDataSize(), ATOM_ADDR);
            d.reset();
        }
    }
    //d.isReady Should Be TRUE right now
}


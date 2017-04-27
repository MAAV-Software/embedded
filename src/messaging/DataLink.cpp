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
#include "messaging/DataLink.hpp"
#include "messaging/Decoder.hpp"
#include "messaging/MessageHandler.hpp"
#include "messaging/TransmitHandler.hpp"
#include "messaging/feedback_t.h"
#include "messaging/lidar_t.h"
#include "messaging/imu_t.h"
#include "messaging/dji_t.h"

//LCM Sender Address (we don't use UDP, it just has to be something)
static const uint64_t ATOM_ADDR = 9001;

using namespace std;

DataLink::DataLink(void (*f)(const uint8_t*, uint32_t))
{
    //Initialize LCM and handlers
	msgSender = TransmitHandler(f);
    lcmlite_init(&lcm, transmitPacket, &msgSender);

    //Initialize LCM subscriptions
    djiSub.callback = callback;
    djiSub.channel = "DJI";
    djiSub.user = &msgHandler;

    //Subscribe LCM subscriptions
    lcmlite_subscribe(&lcm, &djiSub);
}

void DataLink::send(emergency_t *msg)
{
    char buf[128];
    int len = emergency_t_encode(buf, 0, 128, msg);
	lcmlite_publish(&lcm, "EMS", buf, len);
}

void DataLink::send(feedback_t *msg)
{
    char buf[128];
    int len = feedback_t_encode(buf, 0, 128, msg);
	lcmlite_publish(&lcm, "FEB", buf, len);
}

void DataLink::send(lidar_t *msg)
{

	char buf[128];
	int len = lidar_t_encode(buf, 0, 128, msg);
	lcmlite_publish(&lcm, "LID", buf, len);

}

void DataLink::send(imu_t *msg)
{

	char buf[256];
	int len = imu_t_encode(buf, 0, 256, msg);
	lcmlite_publish(&lcm, "IMU", buf, len);

}

void DataLink::processRecv(const uint8_t raw)
{
	d.push(raw);

	if(d.isError()) d.reset(); // check for error

	if(d.isDone()) // check for done
	{
		lcmlite_receive_packet(&lcm, d.packetData(), d.packetDataSize(), ATOM_ADDR);
		d.reset();
	}
}


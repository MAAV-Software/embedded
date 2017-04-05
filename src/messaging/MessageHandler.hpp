#ifndef MESSAGEHANDLER_HPP_
#define MESSAGEHANDLER_HPP_

#include "messaging/lcmlite.h"
#include "messaging/dji_t.h"

struct MessageHandler
{
	dji_t dji;

	MessageHandler();
};

void callback(lcmlite_t *lcm, const char *channel, const void *buf, 
			  int buf_len, void *user);

#endif // MessageHandler.hpp

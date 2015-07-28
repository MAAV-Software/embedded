#ifndef MESSAGEHANDLER_HPP_
#define MESSAGEHANDLER_HPP_

#include "messaging/lcmlite.h"
#include "messaging/gains_t.h"
#include "messaging/raw_pose_t.h"
#include "messaging/setpt_t.h"


struct MessageHandler
{
	gains_t gains;
	raw_pose_t rawPose;
	setpt_t setpt;
};


void callback(lcmlite_t *lcm, const char *channel, const void *buf, 
					int buf_len, void *user);

#endif // MessageHandler.hpp

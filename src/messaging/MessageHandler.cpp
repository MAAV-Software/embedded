#include <cstring>
#include "messaging/MessageHandler.hpp"

using std::strncmp;

MessageHandler::MessageHandler()
{
	gains.utime = 0;
	for (uint8_t i = 0; i < 6; ++i)
		gains.xGains[i] = gains.yGains[i] = gains.zGains[i] = 0;
	for (uint8_t i = 0; i < 3; ++i)
		gains.yawGains[i] = 0;

	setpt.flags = setpt.setptType = 0;
	setpt.utime = 0;
	setpt.x = setpt.y = setpt.yaw = setpt.z = 0;

	rawPose.utime = 0;
	rawPose.x = rawPose.y = rawPose.yaw = 0;
}

void callback(lcmlite_t *lcm, const char *channel, const void *buf, int buf_len, void *user)
{
	MessageHandler *mh = (MessageHandler*)user;
    if(strncmp(channel, "GNS", 3) == 0) 
		gains_t_decode(buf, 0, buf_len, &(mh->gains));
    else if(strncmp(channel, "RWP", 3) == 0)
        raw_pose_t_decode(buf, 0, buf_len, &(mh->rawPose));
    else if(strncmp(channel, "SET", 3) == 0)
        setpt_t_decode(buf, 0, buf_len, &(mh->setpt));
}


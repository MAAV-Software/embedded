#include <cstring>
#include "messaging/MessageHandler.hpp"

using std::strncmp;

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


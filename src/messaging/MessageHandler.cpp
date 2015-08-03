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

	setpt.flags = 0;
	setpt.utime = 0;
	setpt.x = setpt.y = setpt.yaw = setpt.z = 0;

	rawPose.utime = 0;
	rawPose.x = rawPose.y = rawPose.yaw = 0;
}

void callback(lcmlite_t *lcm, const char *channel, const void *buf, int buf_len, void *user)
{
    uint8_t alecBuf[] = {0xce, 0x34, 0x35, 0x14, 0x15,
                         0xd5, 0x26, 0x2d, 0x41, 0x20,
                         0x00, 0x00, 0x41, 0xa0, 0x00,
                         0x00, 0x41, 0xf0, 0x00, 0x00,
                         0x42, 0x20, 0x00, 0x00, 0x01,
                         0x00, 0x00, 0x00, 0x32};
    uint8_t mybuf[29];
    for (int i = 0; i < buf_len; ++i)
        mybuf[i] = ((uint8_t*)buf)[i];

    setpt_t alecTest;
    int err1 = setpt_t_decode(alecBuf, 0, 29, &alecTest);

    setpt_t original;
    original.x = 10;
    original.y = 20;
    original.z = 30;
    original.yaw = 40;
    original.utime = 50;
    original.flags = 1;

    uint8_t origBuf[128];
    int err2 = setpt_t_encode(origBuf, 0, 128, &original);

    setpt_t myTest;
    int testLen = setpt_t_encoded_size(&original);
    int err3 = setpt_t_decode(origBuf, 0, testLen, &myTest);
    uint8_t dummy = 0;

    ////////////////////////////////////////////////
	MessageHandler *mh = (MessageHandler*)user;
    if (strncmp(channel, "GNS", 3) == 0)
		gains_t_decode(buf, 0, buf_len, &(mh->gains));
    else if (strncmp(channel, "RWP", 3) == 0)
        raw_pose_t_decode(buf, 0, buf_len, &(mh->rawPose));
    else if (strncmp(channel, "SET", 3) == 0)
        setpt_t_decode(buf, 0, buf_len, &(mh->setpt));
}

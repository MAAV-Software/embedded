#include "seriallcm.h"
#include <iostream>

SerialLCM::SerialLCM(char *tty_filename) : tty(tty_filename)
{
	lcmlite_init(&lcm, transmission_function, (void*)&tty);
}

void SerialLCM::transmission_function(const void *_buf, int buf_len, void *user)
{
	// Get our serial port back from void* user
	SerialTTY *port;
	port = (SerialTTY*)user;
	// Send the data
	port->send((char*)_buf, buf_len);
}

void SerialLCM::internal_feedback_handler(lcmlite_t *lcm, const char *channel, const void *buf, int buf_len, void *user)
{
	// decode the packet
	feedback_t fbk;
	feedback_t_decode(buf, 0, buf_len, &fbk);
	// cast user back to the handler pointer
	fbk_handler_t handler = (fbk_handler_t)user;
	// call the handler
	handler(&fbk);
}

void SerialLCM::internal_dof_feedback_handler(lcmlite_t *lcm, const char *channel, const void *buf, int buf_len, void *user)
{
	// decode the packet
	dof_feedback_t fbk;
	dof_feedback_t_decode(buf, 0, buf_len, &fbk);
	// cast user back to the handler pointer
	doffbk_handler_t handler = (doffbk_handler_t)user;
	// call the handler
	handler(&fbk);
}

void SerialLCM::internal_dji_feedback_handler(lcmlite_t *lcm, const char *channel, const void *buf, int buf_len, void *user)
{
	// decode the packet
	djiout_feedback_t fbk;
	djiout_feedback_t_decode(buf, 0, buf_len, &fbk);
	// cast user back to the handler pointer
	djifbk_handler_t handler = (djifbk_handler_t)user;
	// call the handler
	handler(&fbk);
}

void SerialLCM::internal_str_log_handler(lcmlite_t *lcm, const char *channel, const void *buf, int buf_len, void *user)
{
	// decode the packet
	str_log_t log;
	str_log_t_decode(buf, 0, buf_len, &log);
	// cast user back to the handler pointer
	strlog_handler_t handler = (strlog_handler_t)user;
	// call the handler
	handler(&log);
}

void SerialLCM::send(target_t *message)
{
	// encode the message
	char message_buf[128];
	int message_len;
	message_len = target_t_encode(message_buf, 0, 128, message);
	// publish the message
	lcmlite_publish(&lcm, CHANNEL_TARGET, message_buf, message_len);
}

void SerialLCM::send(tuning_t *message)
{
	// encode the message
	char message_buf[128];
	int message_len;
	message_len = tuning_t_encode(message_buf, 0, 128, message);
	// publish the message
	lcmlite_publish(&lcm, CHANNEL_TUNING, message_buf, message_len);
}

void SerialLCM::send(position_t *message)
{
	// encode the message
	char message_buf[128];
	int message_len;
	message_len = position_t_encode(message_buf, 0, 128, message);
	// publish the message
	lcmlite_publish(&lcm, CHANNEL_POSITION, message_buf, message_len);
}

void SerialLCM::set_feedback_handler(fbk_handler_t handler)
{
	fbk_handler = handler;
	feedback_sub.channel = CHANNEL_FEEDBACK;
	feedback_sub.callback = internal_feedback_handler;
	feedback_sub.user = (void*)fbk_handler;
	lcmlite_subscribe(&lcm, &feedback_sub);
}

void SerialLCM::set_dof_feedback_handler(doffbk_handler_t handler)
{
	doffbk_handler = handler;
	dof_feedback_sub.channel = CHANNEL_DOF_FEEDBACK;
	dof_feedback_sub.callback = internal_dof_feedback_handler;
	dof_feedback_sub.user = (void*)doffbk_handler;
	lcmlite_subscribe(&lcm, &dof_feedback_sub);
}

void SerialLCM::set_dji_feedback_handler(djifbk_handler_t handler)
{
	djifbk_handler = handler;
	dji_feedback_sub.channel = CHANNEL_DJI_FEEDBACK;
	dji_feedback_sub.callback = internal_dji_feedback_handler;
	dji_feedback_sub.user = (void*)djifbk_handler;
	lcmlite_subscribe(&lcm, &dji_feedback_sub);
}

void SerialLCM::set_str_log_handler(strlog_handler_t handler)
{
	strlog_handler = handler;
	str_log_sub.channel = CHANNEL_STR_LOG;
	str_log_sub.callback = internal_str_log_handler;
	str_log_sub.user = (void*)strlog_handler;
	lcmlite_subscribe(&lcm, &str_log_sub);
}

void SerialLCM::receive_and_process()
{
	// Read from tty
	// If there is an entire packet to handle, handle it
	while(tty.receive_and_process())
	{
		char packet[256];
		int len = tty.last_packet(packet);
		lcmlite_receive_packet(&lcm, packet, len, FROM_ADDR);
	}
}

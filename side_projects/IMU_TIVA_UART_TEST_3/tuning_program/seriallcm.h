#ifndef SERIAL_LCM_H
#define SERIAL_LCM_H

#include "data_link.h"
#include "serial.h"
#include "lcmlite.h"
#include "target_t.h"
#include "tuning_t.h"
#include "position_t.h"
#include "feedback_t.h"
#include "dof_feedback_t.h"
#include "djiout_feedback_t.h"
#include "str_log_t.h"

typedef void (*fbk_handler_t)(feedback_t *);
typedef void (*doffbk_handler_t)(dof_feedback_t *);
typedef void (*djifbk_handler_t)(djiout_feedback_t *);
typedef void (*strlog_handler_t)(str_log_t *);

class SerialLCM
{
		// lcm struct
		lcmlite_t lcm;
		// fnptr to the function we call when we get a message
		fbk_handler_t fbk_handler;
		doffbk_handler_t doffbk_handler;
		djifbk_handler_t djifbk_handler;
		strlog_handler_t strlog_handler;
		// subscription for lcmlite for feedback
		lcmlite_subscription_t feedback_sub;
		lcmlite_subscription_t dof_feedback_sub;
		lcmlite_subscription_t dji_feedback_sub;
		lcmlite_subscription_t str_log_sub;
		// handler functions for lcmlite to call when it gets a messages
		static void internal_feedback_handler(lcmlite_t *lcm, const char *channel, const void *buf, int buf_len, void *user);
		static void internal_dof_feedback_handler(lcmlite_t *lcm, const char *channel, const void *buf, int buf_len, void *user);
		static void internal_dji_feedback_handler(lcmlite_t *lcm, const char *channel, const void *buf, int buf_len, void *user);
		static void internal_str_log_handler(lcmlite_t *lcm, const char *channel, const void *buf, int buf_len, void *user);
		// transmission function for lcmlite to call for sending
		static void transmission_function(const void *_buf, int buf_len, void *user);

	public:
		// serial port to be written and read from
		SerialTTY tty;
		/**
		 * @brief call this regularly.
		 * @details reads any new data in from the serial port and
		 * processes the data. If a packet comes in, it calls the
		 * appropriate handler. 
		 */
		void receive_and_process();
		/**
		 * @brief sends a target message
		 * @param message the message to send
		 */
		void send(target_t *message);
		/**
		 * @brief sends a tuning message
		 * @param message the message to send
		 */
		void send(tuning_t *message);
		/**
		 * @brief sends a position message
		 * @param message the message to send
		 */
		void send(position_t *message);
		/**
		 * @brief sets a feedback handler
		 * @details this function will enable SerialLCM to handle
		 * feedback messages. When a feedback message is received
		 * SerialLCM will decode the message into a feedback_t
		 * struct and pass it into the handler
		 * @param handler function pointer to the function to call
		 * when a feedback message is received
		 */
		void set_feedback_handler(fbk_handler_t handler);
		
		/**
		 * @brief sets a dof feedback tuning message handler. 
		 * @details this function will enable SerialLCM to handle
		 * dof feedback tuning messages. After this is set, when a dof
		 * feedback message is received, SerialLCM will decode the 
		 * message into a dof_feedback_t struct and pass it into the 
		 * handler
		 * @param handler function pointer to the function to call when
		 * a dof feedback message is received
		 */
		void set_dof_feedback_handler(doffbk_handler_t handler);

		/**
		 * @brief sets a dji feedback tuning message handler. 
		 * @details this function will enable SerialLCM to handle
		 * dji feedback tuning messages. After this is set, when a dji
		 * feedback message is received, SerialLCM will decode the 
		 * message into a dji_feedback_t struct and pass it into the 
		 * handler
		 * @param handler function pointer to the function to call when
		 * a dji feedback message is received
		 */
		void set_dji_feedback_handler(djifbk_handler_t handler);

		/**
		 * @brief sets a string logging message handler. 
		 * @details this function will enable SerialLCM to handle
		 * string logging messages. After this is set, when a string
		 * logging message is received, SerialLCM will decode the 
		 * message into a str_log_t struct and pass it into the 
		 * handler
		 * @param handler function pointer to the function to call when
		 * a string logging message is received
		 */
		void set_str_log_handler(strlog_handler_t handler);

		/**
		 * @brief Constructs a SerialLCM object that reads/writes from the
		 * serial port (or any file) named by tty_filename;
		 * @param tty_filename filename of the serial port to hook on to
		 */
		SerialLCM(char *tty_filename);

};
#endif

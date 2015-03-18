#ifndef SERIAL_H
#define SERIAL_H
#include <termios.h>
#include "lcmlite.h"
#include "data_link.h"
#include "target_t.h"
#include "tuning_t.h"
#include "position_t.h"
#include "feedback_t.h"

class SerialTTY
{
		// termios struct for tty settings
		struct termios tio;
		// fd of serial port to use
		int serialtty;
		// data frame
		data_frame_t *frame;
		// last packet
		char *last;
		// last len
		int lastsz;
 	public:
		// Constructor
		SerialTTY(char * tty_filename);
		// Send len char buffer
		void send(char * buffer, int len);
		/** 
		 * @brief do not use unless you know what you are doing
		 * @details Used internally by SerialTTY, this
		 * function performs a nonblocking read of a byte from
		 * the serial port. If a character is received it is
		 * stored to to and true is returned. If no character is
		 * received, the value in to is not defined, and will be
		 * whatever character the system implementation of read(2)
		 * stores in the destination buffer when a read fails
		 * Does not process received bytes in any way. Calling
		 * this will likely cause receive_and_process() to fail,
		 * however, this is made public for testing and debug.
		 * @param to place where the character will be stored
		 */
		bool recv(char &to);
		/**
		 * @brief do not use unless you know what you are doing
		 * @details Receives a character. Blocks until a character
		 * is available to be read. For testing and debug purposes
		 * only.
		 */
		char blockrecv();
		/**
		 * @brief call this regularly
		 * @details the function reads data from the serial port
		 * if a complete packet is received, it will return true.
		 * After that, the entire contents of the packet will be 
		 * available by calling last_packet(char*). If a complete
		 * packet was not received, it will return false. This
		 * function handles all receiving errors by itself. 
		 * @return true when a complete packet has been received,
		 * otherwise returns false
		 */
		bool receive_and_process();
		/**
		 * @brief get the last packet received
		 * @details copies the buffer of the last packet into to 
		 * and returns the length of the buffer
		 * @param where to put the last packet received data
		 * @return the length of the data in to
		 */
		int last_packet(char *to);
};
#endif

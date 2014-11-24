/*
 * messaging.c
 *
 *  Created on: Nov 20, 2014
 *      Author: Sasawat
 */


#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "ringbuf.h"
#include "protobuf.h"
#include "messaging.h"

messaging_t *mdat;

ringbuf_t ringbuf;

uint8_t peekbuf[3];

uint8_t mbuf[250];


/*
 * Initializes messaging with messagedata as the location for messages to be put after decode.
 *
 * \param messagedata pointer to a messaging_t struct in which the information from decoded messages will be put
 */
void messaging_init(messaging_t *messagedata)
{
	mdat = messagedata;
	ringbuf_init(&ringbuf);
}

/*
 * Interrupt Handler for receiving control messages over UART
 * Should be bound to UART RX Interrupt and RT Interrupt to ensure all bytes get processed
 */
void messaging_UARTRxIntHandler(void)
{
	//Cleaer the interrupt flag
	uint32_t status;
	status = UARTIntStatus(MESSAGING_UART_BASE, true);
	UARTIntClear(MESSAGING_UART_BASE, status);

	//Dump HW FIFO bytes into ring buffer
	while (UARTCharsAvail(MESSAGING_UART_BASE) &&
			ringbuf_push(&ringbuf, UARTCharGet(MESSAGING_UART_BASE)));
	//We should probably handle the case that the buffer gets full
	//Somehow.

	//See if a message can be read out of the ring buffer
	//First check is whether it could possibly contain a valid message
	int32_t unread = ringbuf_unread(&ringbuf);
	if(unread >= MESSAGING_MIN_VALID_MESSAGE_SIZE)
	{
		//Next check whether the message can be read out based on length
		peekbuf[0] = peekbuf[1] = peekbuf[2] = 0;
		ringbuf_peek(
				peekbuf,
				&ringbuf,
				MESSAGING_SIZE_DELIMIT_MAX_LEN);
		int32_t msgLen =  Message_get_delimited_size(peekbuf,0);
		if(Message_can_read_delimited_from(
				peekbuf,
				0,
				unread))
		{
			//Read the message
			ringbuf_popMany(mbuf, &ringbuf, msgLen);
			all_read_delimited_from(mbuf, mdat, 0);
		}
	}
}

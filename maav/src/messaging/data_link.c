#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

// Uncomment to make the interrupt service handler turn on the red LED for debug purposes
// #define ISR_LED

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"

#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "driverlib/gpio.h"

#include "messaging/data_link.h"
#include "messaging/ringbuf.h"
#include "messaging/lcmlite.h"
#include "messaging/feedback_t.h"
#include "messaging/position_t.h"
#include "messaging/target_t.h"
#include "messaging/tuning_t.h"
#include "messaging/djiout_feedback_t.h"
#include "messaging/dof_feedback_t.h"
#include "messaging/str_log_t.h"

static ringbuf_t ringbuf;
static data_frame_t *messaging_frame;
//static lcmlite_subscription_t sub_feedback;
static lcmlite_subscription_t sub_position;
static lcmlite_subscription_t sub_target;
static lcmlite_subscription_t sub_tuning;
// for lcmlite transmit handler function
uint8_t ringbuf_internal_buffer[256];
char transmit_user[256];
static lcmlite_t lcm;

char* data_frame_state_names[] = {
		"READY",
		"LEN_1",
		"LEN_2",
		"LEN_1_ESCP",
		"LEN_2_ESCP",
		"READ",
		"READ_ESCP",
		"CHECKSUM",
		"CHECKSUM_ESCP",
		"DONE",
		"START_ERR",
		"CHECKSUM_ERR",
		"DATA_ERR" };
char* feedback_channel_name = CHANNEL_FEEDBACK;
char* position_channel_name = CHANNEL_POSITION;
char* target_channel_name = CHANNEL_TARGET;
char* tuning_channel_name = CHANNEL_TUNING;

static inline void data_link_branchless_assemble_byte(uint8_t* pkt, size_t* pkt_i,
	uint8_t data_byte);

static inline uint8_t data_link_branchless_decode_byte(uint8_t* pkt, size_t* pkt_i);

data_frame_t* data_frame_create(uint16_t size)
{
	data_frame_t* frame = (data_frame_t*) malloc(sizeof(data_frame_t));
	frame->buffer = (uint8_t*) malloc(sizeof(uint8_t) * size);
	frame->size = 0;
	frame->index = 0;
	frame->state = READY;

	return frame;
}

void target_callback(lcmlite_t *from, const char *channel,
		const void *buf, int buf_len, void *user)
{
	// Get target_t to decode to
	target_t *to = (target_t*)user;
	// Decode
	target_t_decode(buf, 0, buf_len, to);

}
void tuning_callback(lcmlite_t *from, const char *channel,
		const void *buf, int buf_len, void *user)
{
	// Get tuning_t to decode to
	tuning_t *to = (tuning_t*)user;
	// Decode
	tuning_t_decode(buf, 0, buf_len, to);

}
void position_callback(lcmlite_t *from, const char *channel,
		const void *buf, int buf_len, void *user)
{
	// Get position_t to decode to
	position_t *to = (position_t*)user;
	// Decode
	position_t_decode(buf, 0, buf_len, to);

}

void data_link_lcmlite_transmit_packet_handler(const void *_buf, int buf_len,
		void *user)
{

	/*int16_t len = data_link_assemble_packet((uint8_t*)_buf, (uint8_t*)transmit_user, buf_len);
	int i = 0;
	for(i = 0; i < len; i++)
	{
		uint8_t sendtestvar = (uint8_t)transmit_user[i];
		ringbuf_push(&ringbuf, sendtestvar);
		transmit_user[i] = 0;
	}*/// For SELF_TEST
	//Assembly the packet
	uint8_t *sendbuffer = (uint8_t*)user;
	int16_t len = data_link_assemble_packet(
			(uint8_t*)_buf,
			sendbuffer,
			buf_len);
	//Send the packet
	int i = 0;
	for (i = 0; i < len; ++i) {
		unsigned char test = sendbuffer[i];
		UARTCharPut(DATA_LINK_UART_BASE, test);
	}
}

void data_link_send_feedback(feedback_t *message)
{
	//Encode
	uint8_t encoded[256];
	int len = feedback_t_encode(encoded, 0, 256, message);
	//Publish
	lcmlite_publish(&lcm, CHANNEL_FEEDBACK, encoded, len);
}

void data_link_send_dof_feedback(dof_feedback_t *message)
{
	//Encode
	uint8_t encoded[256];
	int len = dof_feedback_t_encode(encoded, 0, 256, message);
	//Publish
	lcmlite_publish(&lcm, CHANNEL_DOF_FEEDBACK, encoded, len);
}

void data_link_send_djiout_feedback(djiout_feedback_t *message)
{
	//Encode
	uint8_t encoded[256];
	int len = djiout_feedback_t_encode(encoded, 0, 256, message);
	//Publish
	lcmlite_publish(&lcm, CHANNEL_DJI_FEEDBACK, encoded, len);
}

static void data_link_send_string_log_unsafe(char *message, int32_t time)
{
	//Make message
	str_log_t strlog_message;
	strlog_message.timestamp = time;
	strlog_message.mess = message;
	//Encode
	uint8_t encoded[256];
	int len = str_log_t_encode(encoded, 0, 256, &strlog_message);
	//Publish
	lcmlite_publish(&lcm, CHANNEL_STR_LOG, encoded, len);	
}

void data_link_send_string_log(char *message, int32_t time)
{
	// Be safe
	message[120] = '\0';
	// Send
	data_link_send_string_log_unsafe(message, time);
}

void data_link_send_long_string_log(char *message, int32_t time)
{
	//Loop through, chunk the messages when appropriate
	char *loc = message;
	int at = 0;
	while(loc[at])
	{
		//If we have a max size string, chunk it and send
		if(at == 120)
		{
			//Preserve the 120th byte
			char temp = loc[120];
			data_link_send_string_log(loc, time);
			//Restore the 120th byte
			loc[120] = temp;
			loc = loc + 120;
			at = 0;
		}
		else{
			++at;
		}
	}
	data_link_send_string_log(loc, time);
}

void data_link_process_incoming()
{
	// Loop through and process all received data
	while (ringbuf_unread(&ringbuf)) {
		// Get bytes from ring buffer and push them into messaging frame
		uint8_t testvar = ringbuf_pop(&ringbuf);
		data_frame_push_byte(messaging_frame, testvar);
		// Check messaging frame if it has recieved an entire data_link packet
		if (messaging_frame->state == DONE) {
			// Forward the packet to lcmlite for handling
			lcmlite_receive_packet(&lcm,
					messaging_frame->buffer,
					messaging_frame->size,
					FROM_ADDR);
			// Clear the frame to prepare for next packet
			data_frame_clear(messaging_frame);
		} else if ( // Check messaging frame for errors
				messaging_frame->state == START_ERR ||
				messaging_frame->state == DATA_ERR) {
			data_frame_clear(messaging_frame);
			//TODO: Is just dropping packets enough for error handling?
		}
	}
}

void data_link_init(position_t *position, target_t *target, tuning_t *tuning)
{
	//Init ringbuffer and message frame
	ringbuf_init_static(&ringbuf, ringbuf_internal_buffer, 256);
	messaging_frame = data_frame_create(256);

	//Init LCM
	lcmlite_init(&lcm, data_link_lcmlite_transmit_packet_handler, transmit_user);

	//Setting up subscriptions
	sub_position.channel = position_channel_name;
	sub_position.user = position;
	sub_position.callback = position_callback;

	sub_target.channel = target_channel_name;
	sub_target.user = target;
	sub_target.callback = target_callback;

	sub_tuning.channel = tuning_channel_name;
	sub_tuning.user = tuning;
	sub_tuning.callback = tuning_callback;

	//Subscribe
	lcmlite_subscribe(&lcm, &sub_position);
	lcmlite_subscribe(&lcm, &sub_target);
	lcmlite_subscribe(&lcm, &sub_tuning);
}

void data_link_uart_rx_isr(void)
{
	//Cleaer the interrupt flag
	uint32_t status;
	status = UARTIntStatus(DATA_LINK_UART_BASE, true);
	UARTIntClear(DATA_LINK_UART_BASE, status);

	//Dump HW FIFO bytes into ring buffer
	while (UARTCharsAvail(DATA_LINK_UART_BASE) &&
			ringbuf_push(&ringbuf, UARTCharGet(DATA_LINK_UART_BASE)));

	// Lights up the red LED when the interrupt triggers.
	// Useful for checking whether UART/Interrupt settings are correct
#ifdef ISR_LED
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 255);
#endif
}

void data_frame_destroy(data_frame_t* frame)
{
	free(frame->buffer);
	free(frame);
}

void data_frame_clear(data_frame_t* frame)
{
	frame->size = 0;
	frame->index = 0;
	frame->state = READY;
}

// state machine to process bytes in the data link layer
// TODO: try to make branchless
// TODO: check size
bool data_frame_push_byte(data_frame_t* frame, uint8_t byte)
{
	switch (frame->state) {
		case READY:
			if (byte != DATA_FRAME_START_DELIMITER) {
				frame->state = START_ERR;
				return false;
			} else {
				frame->state = LEN_1;
			}
			break;
		case LEN_1:
			if (byte == DATA_FRAME_ESCAPE_CHAR) {
				frame->state = LEN_1_ESCP;
			} else {
				frame->size = (uint16_t) byte << 8;
				frame->state = LEN_2;
			}
			break;
		case LEN_2:
			if (byte == DATA_FRAME_ESCAPE_CHAR) {
				frame->state = LEN_2_ESCP;
			} else {
				frame->size |= byte & 0xFF;
				frame->state = READ;
			}
			if (frame->size == 0) {
				frame->state = DONE;
			}
			break;
		case LEN_1_ESCP:
			frame->size = (uint16_t) (byte ^ DATA_FRAME_XOR) << 8;
			frame->state = LEN_2;
			break;
		case LEN_2_ESCP:
			frame->size |= (byte ^ DATA_FRAME_XOR) << 8;
			frame->state = READ;
			if (frame->size == 0) {
				frame->state = DONE;
			}
			break;
		case READ:
			if (byte == DATA_FRAME_ESCAPE_CHAR) {
				frame->state = READ_ESCP;
			} else {
				(frame->buffer)[frame->index] = byte;
				frame->checksum += byte;
				frame->index++;
				if (frame->index == frame->size) {
					frame->state = CHECKSUM;
				}
			}
			break;
		case READ_ESCP:
			(frame->buffer)[frame->index] = byte ^ DATA_FRAME_XOR;
			frame->checksum += byte ^ DATA_FRAME_XOR;
			frame->index++;
			if (frame->index == frame->size) {
				frame->state = CHECKSUM;
			} else {
				frame->state = READ;
			}
			break;
		case CHECKSUM:
				frame->state = DONE;
			break;
		case CHECKSUM_ESCP:
				frame->state = DONE;
			break;
		default:
			return false;
	}
	return true;
}

uint16_t data_link_assemble_packet(uint8_t* msg, uint8_t* pkt, uint16_t size)
{
	size_t msg_i = 0, pkt_i = 0;
	pkt[pkt_i++] = DATA_FRAME_START_DELIMITER;

	data_link_branchless_assemble_byte(pkt, &pkt_i, size >> 8);
	pkt_i++;
	data_link_branchless_assemble_byte(pkt, &pkt_i, size & 0xFF);
	pkt_i++;

	uint8_t checksum = 0;
	while (msg_i < size) {
		data_link_branchless_assemble_byte(pkt, &pkt_i, msg[msg_i]);
		checksum += msg[msg_i++];
		pkt_i++;
	}

	checksum = 0xFF - checksum;
	data_link_branchless_assemble_byte(pkt, &pkt_i, checksum);
	return pkt_i + 1;
}

bool data_link_decode_packet(uint8_t* msg, uint8_t* pkt, uint16_t* size)
{
	size_t msg_i = 0, pkt_i = 0;
	if (pkt[pkt_i++] != DATA_FRAME_START_DELIMITER) {
		return false;
	}

	uint8_t size_upper = data_link_branchless_decode_byte(pkt, &pkt_i);
	pkt_i++;
	uint8_t size_lower = data_link_branchless_decode_byte(pkt, &pkt_i);
	pkt_i++;
	*size = (uint16_t) (size_upper << 8) | size_lower;

	uint8_t checksum = 0;
	while (msg_i < *size) {
		msg[msg_i] = data_link_branchless_decode_byte(pkt, &pkt_i);
		checksum += msg[msg_i++];
		pkt_i++;
	}

	return true;
}

static inline void data_link_branchless_assemble_byte(uint8_t* pkt, size_t* pkt_i,
	uint8_t data_byte)
{
	uint8_t is_special_char = (data_byte == DATA_FRAME_START_DELIMITER) +
	(data_byte == DATA_FRAME_ESCAPE_CHAR);
	uint8_t xor_val = is_special_char << 5;
	pkt[*pkt_i] = DATA_FRAME_ESCAPE_CHAR;
	pkt[*pkt_i + is_special_char] = data_byte ^ xor_val;
	*pkt_i += is_special_char;
}

static inline uint8_t data_link_branchless_decode_byte(uint8_t* pkt, size_t* pkt_i)
{
	uint8_t is_delimited = (pkt[*pkt_i] == DATA_FRAME_ESCAPE_CHAR);
	uint8_t xor_val = is_delimited << 5;
	*pkt_i += is_delimited;
	return (pkt[*pkt_i] ^ xor_val);
}

void data_link_test_insert_ringbuf(uint8_t byte)
{
	ringbuf_push(&ringbuf, byte);
}


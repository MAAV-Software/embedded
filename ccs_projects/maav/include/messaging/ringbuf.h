/*
 * ringbuf.h
 *
 *  Created on: Oct 27, 2014
 *      Author: Sasawat
 */

#ifndef RINGBUF_H_
#define RINGBUF_H_

#include <stdint.h>
#include <stdbool.h>

//TODO: Determine optimal ringbuffer size for messaging
#define RINGBUF_MAX_SIZE 300

typedef struct ringbuf
{
	uint8_t *buffer;
	uint8_t *end;
	uint8_t *reader;
	uint8_t *writer;
}ringbuf_t;

/// Write datum to the end of the ringbuffer.
/// Returns remaining space
extern int32_t ringbuf_push(ringbuf_t *ringbuffer, uint8_t datum);

/// Writes len number of bytes in data to ringbuffer
/// Returns remaining space
extern int32_t ringbuf_pushMany(ringbuf_t *ringbuffer, uint8_t *data, int32_t len);

/// Reads one byte from ringbuffer, advancing the reading head
extern uint8_t ringbuf_pop(ringbuf_t *ringbuffer);

/// Reads len bytes to popTo from ringbuffer, advancing the reading head
extern int32_t ringbuf_popMany(uint8_t *popTo, ringbuf_t *ringbuffer, int32_t len);

/// Reads len bytes to peekTo from ringbuffer, without advancing the reading head
extern void ringbuf_peek(uint8_t *peekTo, ringbuf_t *ringbuffer, int32_t len);

/// Returns the space left in ringbuffer to write to
extern int32_t ringbuf_spaceLeft(ringbuf_t *ringbuffer);

/// Returns the number of unread bytes in ringbuffer to read
extern int32_t ringbuf_unread(ringbuf_t *ringbuffer);

/// Clears ringbuffer
extern void ringbuf_clear(ringbuf_t *ringbuffer);

/// Initializes ringbuffer
extern void ringbuf_init(ringbuf_t *ringbuffer);

#endif /* RINGBUF_H_ */

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
#define RINGBUF_MAX_SIZE 250

typedef struct ringbuf
{
	uint8_t *buffer;
	uint8_t *end;
	uint8_t *reader;
	uint8_t *writer;
}ringbuf_t;

extern int32_t ringbuf_push(ringbuf_t *ringbuffer, uint8_t ui8Data);

extern int32_t ringbuf_pushMany(ringbuf_t *ringbuffer, uint8_t *pui8Data, int32_t i32Len);

extern uint8_t ringbuf_pop(ringbuf_t *ringbuffer);

extern int32_t ringbuf_popMany(uint8_t *pui8PopTo, ringbuf_t *ringbuffer, int32_t i32Len);

extern void ringbuf_peek(uint8_t *pui8PeekTo, ringbuf_t *ringbuffer, int32_t i32Len);

extern int32_t ringbuf_spaceLeft(ringbuf_t *ringbuffer);

extern int32_t ringbuf_unread(ringbuf_t *ringbuffer);

extern void ringbuf_clear(ringbuf_t *ringbuffer);

extern void ringbuf_init(ringbuf_t *ringbuffer);

#endif /* RINGBUF_H_ */

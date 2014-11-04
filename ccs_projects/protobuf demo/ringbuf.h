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

typedef struct ringbuf
{
	uint8_t *pui8Buffer;
	uint8_t *pui8BufferEnd;
	uint8_t *pui8Reader;
	uint8_t *pui8Writer;
}ringbuf;

extern bool ringbuf_push(ringbuf *ringbuffer, uint8_t ui8Data);

extern bool ringbuf_pushMany(ringbuf *ringbuffer, uint8_t *pui8Data, int32_t i32Len);

extern uint8_t ringbuf_pop(ringbuf *ringbuffer);

extern uint8_t *ringbuf_popMany(ringbuf *ringbuffer, int32_t i32Len);

extern uint8_t ringbuf_peek(ringbuf *ringbuffer, int32_t i32Len);

extern int32_t ringbuf_spaceLeft(ringbuf *ringbuffer);

extern int32_t ringbuf_unread(ringbuf *ringbuffer);

extern void ringbuf_clear(ringbuf *ringbuffer);

extern ringbuf *ringbuf_init();

#endif /* RINGBUF_H_ */

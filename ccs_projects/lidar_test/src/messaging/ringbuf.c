/*
 * ringbuf.c
 *
 *  Created on: Oct 27, 2014
 *      Author: Sasawat
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "messaging/ringbuf.h"

int32_t ringbuf_push(ringbuf_t *ringbuffer, uint8_t datum)
{
	// Check if there is space left
	int32_t left = ringbuf_spaceLeft(ringbuffer);
	if (!left) {
		return 0;
	}

	// Write the data to the writer
	ringbuffer->writer[0] = datum;
	// Advance the writer
	ringbuffer->writer++;
	// Check for writer at end
	if(ringbuffer->writer == ringbuffer->end)
	{
		ringbuffer->writer = ringbuffer->buffer;
	}

	// Return remaining space
	return --left;
}

int32_t ringbuf_pushMany(ringbuf_t *ringbuffer, uint8_t *data, int32_t len)
{
	int32_t at = 0;
	int32_t left = ringbuf_spaceLeft(ringbuffer);
	// Loop through while space remains
	while (left--) {
		ringbuffer->writer[0] = data[at];
		ringbuffer->writer++;
		at++;
		// Check for writer at end
		if(ringbuffer->writer == ringbuffer->end) {
			ringbuffer->writer = ringbuffer->buffer;
		}
		// Check for done
		if(at == len) {
			return left;
		}

	}
	return 0;

}

uint8_t ringbuf_pop(ringbuf_t *ringbuffer)
{
	// Is there stuff to read?
	if (ringbuffer->reader == ringbuffer->writer) {
		return 0;
	}
	// Read
	uint8_t ui8Ret = ringbuffer->reader[0];
	// Advanced read head
	ringbuffer->reader++;
	// Check for read head at end
	if (ringbuffer->reader == ringbuffer->end) {
		ringbuffer->reader = ringbuffer->buffer;
	}
	// Return read byte
	return ui8Ret;
}

int32_t ringbuf_popMany(uint8_t *popTo, ringbuf_t *ringbuffer, int32_t len)
{
	// Check we have enough bytes to read
	int32_t at = 0;
	if (ringbuf_unread(ringbuffer) < len) {
		return 0;
	}
	// Read
	while (ringbuffer->reader != ringbuffer->writer) {
		// Read from reader and write to destination
		popTo[at] = ringbuffer->reader[0];
		// Advance write position on destination
		at++;
		// Advanced reader
		ringbuffer->reader++;
		// Check for reader at end
		if (ringbuffer->reader == ringbuffer->end) {
			ringbuffer->reader = ringbuffer->buffer;
		}
		// Have we read the bytes requested?
		if (at == len) {
			return ringbuf_unread(ringbuffer);
		}
	}
	return 0;
}

void ringbuf_peek(uint8_t *pui8PeekTo, ringbuf_t *ringbuffer, int32_t len)
{
	// Check for enough bytes unread
	if (len > ringbuf_unread(ringbuffer)) {
		return;
	}
	// Temporary reader
	uint8_t *pui8Temp = ringbuffer->reader;
	int32_t at = 0;
	// Loop through peeking
	while (pui8Temp != ringbuffer->writer) {
		pui8PeekTo[at] = pui8Temp[0];
		at++;
		pui8Temp++;
		if (pui8Temp == ringbuffer->end) {
			pui8Temp = ringbuffer->buffer;
		}
		if (at == len) {
			return;
		}
	}
	return;
}

int32_t ringbuf_spaceLeft(ringbuf_t *ringbuffer)
{
	int32_t i32Ret = ringbuffer->reader - ringbuffer->writer - 1;
	if (i32Ret >= 0) {
		return i32Ret;
	}
	return i32Ret + RINGBUF_MAX_SIZE;
}

int32_t ringbuf_unread(ringbuf_t *ringbuffer)
{
	int32_t i32Ret = ringbuffer->writer - ringbuffer->reader;
	if (i32Ret >= 0) {
		return i32Ret;
	}
	return i32Ret + RINGBUF_MAX_SIZE;
}

void ringbuf_clear(ringbuf_t *ringbuffer)
{
	ringbuffer->reader = ringbuffer->buffer;
	ringbuffer->writer = ringbuffer->buffer;
}

void ringbuf_init(ringbuf_t *ringbuffer)
{
	ringbuffer->buffer = malloc(RINGBUF_MAX_SIZE);
	ringbuffer->end = ringbuffer->buffer+RINGBUF_MAX_SIZE;
	ringbuffer->reader = ringbuffer->buffer;
	ringbuffer->writer = ringbuffer->buffer;
}

void ringbuf_destroy(ringbuf_t *ringbuffer)
{
	free(ringbuffer->buffer);
}

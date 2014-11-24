/*
 * ringbuf.c
 *
 *  Created on: Oct 27, 2014
 *      Author: Sasawat
 */

#include "ringbuf.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

int32_t ringbuf_push(ringbuf_t *ringbuffer, uint8_t datum)
{
	int32_t left = ringbuf_spaceLeft(ringbuffer);
	if(!left)
	{
		return 0;
	}

	ringbuffer->writer[0] = datum;
	ringbuffer->writer++;
	if(ringbuffer->writer == ringbuffer->end)
	{
		ringbuffer->writer = ringbuffer->buffer;
	}
	return left;
}

int32_t ringbuf_pushMany(ringbuf_t *ringbuffer, uint8_t *data, int32_t len)
{
	int32_t at = 0;
	int32_t left = ringbuf_spaceLeft(ringbuffer);
	while(left--)
	{
		ringbuffer->writer[0] = data[at];
		ringbuffer->writer++;
		at++;
		if(ringbuffer->writer == ringbuffer->end)
		{
			ringbuffer->writer = ringbuffer->buffer;
		}
		if(at == len)
		{
			return left;
		}

	}
	return 0;

}

uint8_t ringbuf_pop(ringbuf_t *ringbuffer)
{
	if(ringbuffer->reader == ringbuffer->writer)
	{
		return 0;
	}
	uint8_t ui8Ret = ringbuffer->reader[0];
	ringbuffer->reader++;
	if(ringbuffer->reader == ringbuffer->end)
	{
		ringbuffer->reader = ringbuffer->buffer;
	}
	return ui8Ret;
}

int32_t ringbuf_popMany(uint8_t *popTo, ringbuf_t *ringbuffer, int32_t len)
{
	int32_t at = 0;
	if(ringbuf_unread(ringbuffer) < len)
	{
		return 0;
	}
	while(ringbuffer->reader != ringbuffer->writer)
	{
		popTo[at] = ringbuffer->reader[0];
		at++;
		ringbuffer->reader++;
		if(ringbuffer->reader == ringbuffer->end)
		{
			ringbuffer->reader = ringbuffer->buffer;
		}
		if (at == len)
		{
			return ringbuf_unread(ringbuffer);
		}
	}
	return 0;
}

void ringbuf_peek(uint8_t *pui8PeekTo, ringbuf_t *ringbuffer, int32_t len)
{
	if(len > ringbuf_unread(ringbuffer))
	{
		return;
	}
	uint8_t *pui8Temp = ringbuffer->reader;
	int32_t at = 0;
	while(pui8Temp != ringbuffer->writer)
	{
		pui8PeekTo[at] = pui8Temp[0];
		at++;
		pui8Temp++;
		if(pui8Temp == ringbuffer->end)
		{
			pui8Temp = ringbuffer->buffer;
		}
		if (at == len)
		{
			return;
		}
	}
	return;
}

int32_t ringbuf_spaceLeft(ringbuf_t *ringbuffer) {
	int32_t i32Ret = ringbuffer->reader - ringbuffer->writer - 1;
	if (i32Ret >= 0) {
		return i32Ret;
	}
	return i32Ret + RINGBUF_MAX_SIZE;
}

int32_t ringbuf_unread(ringbuf_t *ringbuffer) {
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

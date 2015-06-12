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

int32_t ringbuf_push(ringbuf_t *ringbuffer, uint8_t ui8Data)
{
	int32_t left = ringbuf_spaceLeft(ringbuffer);
	if(!left)
	{
		return 0;
	}

	ringbuffer->writer[0] = ui8Data;
	ringbuffer->writer++;
	if(ringbuffer->writer == ringbuffer->end)
	{
		ringbuffer->writer = ringbuffer->buffer;
	}
	return left;
}

int32_t ringbuf_pushMany(ringbuf_t *ringbuffer, uint8_t *pui8Data, int32_t i32Len)
{
	int32_t i32At = 0;
	int32_t left = ringbuf_spaceLeft(ringbuffer);
	while(left--)
	{
		ringbuffer->writer[0] = pui8Data[i32At];
		ringbuffer->writer++;
		i32At++;
		if(ringbuffer->writer == ringbuffer->end)
		{
			ringbuffer->writer = ringbuffer->buffer;
		}
		if(i32At == i32Len)
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

int32_t ringbuf_popMany(uint8_t *pui8PopTo, ringbuf_t *ringbuffer, int32_t i32Len)
{
	int32_t i32At = 0;
	if(ringbuf_unread(ringbuffer) < i32Len)
	{
		return 0;
	}
	while(ringbuffer->reader != ringbuffer->writer)
	{
		pui8PopTo[i32At] = ringbuffer->reader[0];
		i32At++;
		ringbuffer->reader++;
		if(ringbuffer->reader == ringbuffer->end)
		{
			ringbuffer->reader = ringbuffer->buffer;
		}
		if (i32At == i32Len)
		{
			return ringbuf_unread(ringbuffer);
		}
	}
	return 0;
}

void ringbuf_peek(uint8_t *pui8PeekTo, ringbuf_t *ringbuffer, int32_t i32Len)
{
	if(i32Len > ringbuf_unread(ringbuffer))
	{
		return;
	}
	uint8_t *pui8Temp = ringbuffer->reader;
	int32_t i32At = 0;
	while(pui8Temp != ringbuffer->writer)
	{
		pui8PeekTo[i32At] = pui8Temp[0];
		i32At++;
		pui8Temp++;
		if(pui8Temp == ringbuffer->end)
		{
			pui8Temp = ringbuffer->buffer;
		}
		if (i32At == i32Len)
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

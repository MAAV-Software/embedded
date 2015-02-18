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

#define RINGBUF_MAX_SIZE 250

bool ringbuf_push(ringbuf *ringbuffer, uint8_t ui8Data)
{
	if(!ringbuf_spaceLeft(ringbuffer))
	{
		return false;
	}

	ringbuffer->pui8Writer[0] = ui8Data;
	ringbuffer->pui8Writer++;
	if(ringbuffer->pui8Writer == ringbuffer->pui8BufferEnd)
	{
		ringbuffer->pui8Writer = ringbuffer->pui8Buffer;
	}
	return true;
}

bool ringbuf_pushMany(ringbuf *ringbuffer, uint8_t *pui8Data, int32_t i32Len)
{
	int32_t i32At = 0;
	while(ringbuf_spaceLeft(ringbuffer))
	{
		ringbuffer->pui8Writer[0] = pui8Data[i32At];
		ringbuffer->pui8Writer++;
		i32At++;
		if(ringbuffer->pui8Writer == ringbuffer->pui8BufferEnd)
		{
			ringbuffer->pui8Writer = ringbuffer->pui8Buffer;
		}
		if(i32At == i32Len)
		{
			return true;
		}

	}
	return false;

}

uint8_t ringbuf_pop(ringbuf *ringbuffer)
{
	if(ringbuffer->pui8Reader == ringbuffer->pui8Writer)
	{
		return 0;
	}
	uint8_t ui8Ret = ringbuffer->pui8Reader[0];
	ringbuffer->pui8Reader++;
	if(ringbuffer->pui8Reader == ringbuffer->pui8BufferEnd)
	{
		ringbuffer->pui8Reader = ringbuffer->pui8Buffer;
	}
	return ui8Ret;
}

uint8_t *ringbuf_popMany(ringbuf *ringbuffer, int32_t i32Len)
{
	uint8_t *pui8Ret = malloc(i32Len);
	int32_t i32At = 0;
	if(ringbuf_unread(ringbuffer) < i32Len)
	{
		return 0;
	}
	while(ringbuffer->pui8Reader != ringbuffer->pui8Writer)
	{
		pui8Ret[i32At] = ringbuffer->pui8Reader[0];
		i32At++;
		ringbuffer->pui8Reader++;
		if(ringbuffer->pui8Reader == ringbuffer->pui8BufferEnd)
		{
			ringbuffer->pui8Reader = ringbuffer->pui8Buffer;
		}
		if (i32At == i32Len)
		{
			return pui8Ret;
		}
	}
	free(pui8Ret);
	return 0;
}

uint8_t ringbuf_peek(ringbuf *ringbuffer, int32_t i32Ahead)
{
	uint8_t *pui8Temp;
	pui8Temp = ringbuffer->pui8Reader + i32Ahead;
	if(pui8Temp < ringbuffer->pui8BufferEnd)
	{
		if (pui8Temp <= ringbuffer->pui8Writer) {
			return ringbuffer->pui8Reader[i32Ahead];
		}
		return 0;
	}
	pui8Temp -= RINGBUF_MAX_SIZE;
	if (pui8Temp <= ringbuffer->pui8Writer) {
		return ringbuffer->pui8Reader[i32Ahead];
	}
	return 0;
}

int32_t ringbuf_spaceLeft(ringbuf *ringbuffer) {
	int32_t i32Ret = ringbuffer->pui8Reader - ringbuffer->pui8Writer - 1;
	if (i32Ret >= 0) {
		return i32Ret;
	}
	return i32Ret + RINGBUF_MAX_SIZE;
}

int32_t ringbuf_unread(ringbuf *ringbuffer) {
	int32_t i32Ret = ringbuffer->pui8Writer - ringbuffer->pui8Reader;
	if (i32Ret >= 0) {
		return i32Ret;
	}
	return i32Ret + RINGBUF_MAX_SIZE;
}

void ringbuf_clear(ringbuf *ringbuffer)
{
	ringbuffer->pui8Reader = ringbuffer->pui8Buffer;
	ringbuffer->pui8Writer = ringbuffer->pui8Buffer;
}

ringbuf *ringbuf_init()
{
	ringbuf *pringbufRet = (ringbuf *)malloc(sizeof(ringbuf));
	pringbufRet->pui8Buffer = malloc(RINGBUF_MAX_SIZE);
	pringbufRet->pui8BufferEnd = pringbufRet->pui8Buffer+RINGBUF_MAX_SIZE;
	pringbufRet->pui8Reader = pringbufRet->pui8Buffer;
	pringbufRet->pui8Writer = pringbufRet->pui8Buffer;
	return pringbufRet;
}

void ringbuf_destroy(ringbuf *ringbuffer)
{
	free(ringbuffer->pui8Buffer);
	free(ringbuffer);
}

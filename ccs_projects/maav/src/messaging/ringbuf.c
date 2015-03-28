/*
 * Simple Ringbuffer.
 * 
 *
 *      Author: Sasawat
 */

#include "messaging/ringbuf.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

bool ringbuf_push(ringbuf_t *ringbuffer, uint8_t to_push)
{
	// Enabling or disabling overwrite on push. If overwrite on push is enabled
	// things will be faster since pushing will be like branchless
#ifdef RINGBUF_NO_OVERWRITE
	if(!ringbuf_spaceLeft(ringbuffer))
	{
		return false;
	}
#endif

	// Push the byte in
	ringbuffer->data[ringbuffer->writer] = to_push;

	// Advance the write head
	ringbuffer->writer++;

	// Warparound the write head
	ringbuffer->writer = ringbuffer->writer & ringbuffer->mask;

	// Success!
	return true;
}

bool ringbuf_pushMany(ringbuf_t *ringbuffer, uint8_t *to_push, uint32_t len)
{
	uint32_t at = 0;

	// Push bytes into the ringbuffer
	while (at < len && ringbuf_push(ringbuffer, to_push[at++]));

	return at == len;
}

uint8_t ringbuf_pop(ringbuf_t *ringbuffer)
{
	// Check that we can read
	if(ringbuffer->reader == ringbuffer->writer)
	{
		return 0;
	}

	// Read the data
	uint8_t ret = ringbuffer->data[ringbuffer->reader];

	// Advance the read head
	ringbuffer->reader++;

	// Warp around the read head
	ringbuffer->reader = ringbuffer->reader & ringbuffer->mask;

	// Return the read data
	return ret;
}

void ringbuf_popMany(ringbuf_t *ringbuffer, uint8_t * to, uint32_t len)
{
	uint32_t at = 0;

	// Loop around reading
	while(ringbuffer->reader != ringbuffer->writer)
	{
		// Write to output buffer
		to[at] = ringbuffer->data[ringbuffer->reader];
		// Advance the output buffer write head
		at++;
		// Advance the read head
		ringbuffer->reader++;
		// Wrapround the read head
		ringbuffer->reader = ringbuffer->reader % ringbuffer->mask;

		// We done?
		if (at == len)
		{
			// We done! Return
			return;
		}
		// We not done, go on
	}
}

int32_t ringbuf_spaceLeft(const ringbuf_t *ringbuffer)
{
	uint32_t ret = ringbuffer->reader - ringbuffer->writer - 1;
	return (ret + ringbuffer->mask + 1) % (ringbuffer->mask + 1);
}

int32_t ringbuf_unread(const ringbuf_t *ringbuffer)
{
	uint32_t ret = ringbuffer->writer - ringbuffer->reader;
	return (ret + ringbuffer-> mask + 1) % (ringbuffer->mask + 1);
}

void ringbuf_clear(ringbuf_t *ringbuffer)
{
	ringbuffer->reader = 0;
	ringbuffer->writer = 0;
}

void ringbuf_init_dynamic(ringbuf_t *ringbuffer, uint32_t size)
{
	ringbuffer->data = (uint8_t *)malloc(size);
	ringbuffer->reader = 0;
	ringbuffer->writer = 0;
	ringbuffer->mask = size - 1;
}

void ringbuf_destroy_dynamic(ringbuf_t *ringbuffer)
{
	free(ringbuffer->data);
}

void ringbuf_init_static(ringbuf_t *ringbuffer, uint8_t *arr, uint32_t size)
{
	ringbuffer->data = arr;
	ringbuffer->reader = 0;
	ringbuffer->writer = 0;
	ringbuffer->mask = size - 1;
}

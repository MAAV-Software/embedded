/*
 * ringbuf.h
 *
 *  Created on: Oct 27, 2014
 *      Author: Sasawat
 */

#ifndef RINGBUF_H_
#define RINGBUF_H_

#define RINGBUF_NO_OVERWRITE

#include <stdint.h>
#include <stdbool.h>

typedef struct ringbuf
{
	uint8_t *data;
	uint32_t writer;
	uint32_t reader;
	uint32_t mask;
} ringbuf_t;

// Pushes a byte into a ringbuffer
// MODIFIES: ringbuffer
extern bool ringbuf_push(ringbuf_t *ringbuffer, uint8_t to_push);

// Pushes len number of bytes into a ringbuffer
// MODIFIES: ringbuffer
extern bool ringbuf_pushMany(ringbuf_t *ringbuffer, uint8_t *to_push, uint32_t len);

// Pops a byte out of the ringbuffer and returns it
// MODIFIES: ringbuffer
extern uint8_t ringbuf_pop(ringbuf_t *ringbuffer);

// Pops len number of bytes out of the ringbuffer into to
// MODIFIES ringbuffer, to
extern void ringbuf_popMany(ringbuf_t *ringbuffer, uint8_t * to, uint32_t len);

// Returns the amount of space left in the ringbuffer
extern int32_t ringbuf_spaceLeft(const ringbuf_t *ringbuffer);

// Returns the number of unread space in the ringbuffer
extern int32_t ringbuf_unread(const ringbuf_t *ringbuffer);

// Resets the ringbuffer to empty
extern void ringbuf_clear(ringbuf_t *ringbuffer);

// Initializes a ringbuffer of size size with dynamic memory internal buffer
// MODIFIES: ringbuffer
// NOTE: SIZE MUST BE A POWER OF TWO, 218, 512, etc.
extern void ringbuf_init_dynamic(ringbuf_t *ringbuffer, uint32_t size);

// Destroys a dynamically initialized ringbuffer
// MODIFIES: ringbuffer
// REQUIRES: ringbuffer was initialized with dynamic memory
extern void ringbuf_destroy_dynamic(ringbuf_t *ringbuffer);

// Initializes a ringbuffer of size size with the internal buffer of arr
// MODIFIES: ringbuffer
// REQUIRES: arr has size of at least size
extern void ringbuf_init_static(ringbuf_t *ringbuffer, uint8_t *arr, uint32_t size);

#endif /* RINGBUF_H_ */

#include <iostream>
#include <stdint.h>
#include "messaging/RingBuffer.hpp"
#include <cstdlib>
#include <cassert>

using namespace std;

void fill(RingBuffer<64> &rb, int seed, int count)
{
	srand(seed);
	for(int i = 0; i < count; ++i) rb.push((uint8_t)(rand() % 256));
}

void check(RingBuffer<64> &rb, int seed, int count, int offset)
{
	srand(seed);
	for(int i = 0; i < offset; ++i) seed = rand() % 256;
	for(int i = offset; i < count; ++i) assert(rb.pop() == (rand() % 256));
}

void check(RingBuffer<64> &rb, int seed, int count)
{
	check(rb, seed, count, 0);
}

int main()
{
	uint8_t backing[64];
	RingBuffer<64> rb(backing);

	// Testing basic functionality 
	assert(rb.unread() == 0);
	assert(rb.unwritten() == 63);
	fill(rb, 5, 63);
	assert(rb.unread() == 63);
	assert(rb.unwritten() == 0);
	check(rb, 5, 63);
	assert(rb.unread() == 0);
	assert(rb.unwritten() == 63);

	// Testing filling then reading a bit then filling some more
	fill(rb, 9001, 63);
	check(rb, 9001, 39);
	fill(rb, 69, 13);
	check(rb, 9001, 63, 39);
	check(rb, 69, 13);

	// Testing empty
	fill(rb, 420, 44);
	rb.clear();
	assert(rb.unwritten() == 63);
	assert(rb.unread() == 0);
	
	cout << "Test Successfull!" << endl;
	return 0;
}

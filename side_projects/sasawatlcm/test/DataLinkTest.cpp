#include <iostream>
#include <cassert>
#include <cstdlib>
#include <stdint.h>

#include "messaging/Encoder.hpp"
#include "messaging/Decoder.hpp"

using namespace std;

void fill(uint8_t* buffer, int seed, int count)
{
	srand(seed);
	for(int i = 0; i < count; ++i) buffer[i] = ((uint8_t)(rand() % 256));
}

void check(const uint8_t* buffer, int seed, int count)
{
	srand(seed);
	for(int i = 0; i < count; ++i) assert(buffer[i] == (rand() % 256));
}

int main(void)
{
	Decoder d;
	Encoder e;
	uint8_t buffer[120];

	// Testing against self
	fill(buffer, 42069, 120);
	e.encode(buffer, 120);
	
	const uint8_t* testptr = e.packet();
	while(!d.isDone())
	{
		d.push(*testptr);
		testptr++;
	}
	testptr = d.packetData();
	assert(d.packetDataSize() == 120);
	check(testptr, 42069, d.packetDataSize());
	
	d.reset();

	d = e.packet();
	assert(d.packetDataSize() == 120);
	check(d.packetData(), 42069, d.packetDataSize());

	d.reset();

	// Testing multiple encode
	fill(buffer, 1337, 120);
	e.encode(buffer, 120);
	
	testptr = e.packet();
	while(!d.isDone())
	{
		d.push(*testptr);
		testptr++;
	}
	testptr = d.packetData();
	assert(d.packetDataSize() == 120);
	check(testptr, 1337, d.packetDataSize());

	d.reset();

	d = e.packet();
	assert(d.packetDataSize() == 120);
	check(d.packetData(), 1337, d.packetDataSize());
	
	d.reset();

	// Testing bad decode data
	assert(d.isReady());
	d.push(55);
	assert(d.isError());
	
	cout << "Test Successful! " << endl;
	return 0;
}

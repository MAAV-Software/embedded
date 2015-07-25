#ifndef LOOP_HPP_
#define LOOP_HPP_

#include <vector>
#include <functional>
#include <stdint.h>
#include "runnables/Runnable.hpp"

#include "SdCard.hpp"

/**
 * @brief Class to handle timing events in a loop
 */

struct Event
{
	Runnable* task;
	int32_t lastTime;
	int32_t period;
};

class Loop
{
public:
	Loop();

	/**
	 * @brief adds an event to be run in the loop
	 *
	 * @param fn function pointer to be run
	 * @param periodMs the period for how often to run the function in milliseconds
	 */
	void addEvent(Runnable* task, int32_t periodMs);

	/**
	 * @brief runs the loop that calls events at set times
	 * will not return
	 */
	void run(SdCard* sdcard);

private:
	uint32_t fileNumber;
	char fileName[15];
	std::vector<Event> _events;

};

#endif /* LOOP_HPP_ */

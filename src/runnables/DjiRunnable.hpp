#ifndef DJIRUNNABLE_HPP_
#define DJIRUNNABLE_HPP_

#include "ProgramState.hpp"
#include "runnables/Runnable.hpp"

class DjiRunnable : public Runnable {
private:
	ProgramState *state;
	uint32_t ppmXdlast;
	uint32_t ppmYdlast;
	uint32_t ppmFzlast;
	uint32_t ppmYawdlast;
public:
	DjiRunnable(ProgramState *pState);
	void run();
};

#endif /* DJIRUNNABLE_HPP_ */

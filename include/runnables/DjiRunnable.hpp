#ifndef DJIRUNNABLE_HPP_
#define DJIRUNNABLE_HPP_

#include "ProgramState.hpp"
#include "runnables/Runnable.hpp"

class DjiRunnable : public Runnable {
private:
	ProgramState *state;
public:
	DjiRunnable(ProgramState *pState);
	void run();
};

#endif /* DJIRUNNABLE_HPP_ */

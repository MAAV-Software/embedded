#ifndef DJIRUNNABLE_HPP_
#define DJIRUNNABLE_HPP_

#include "ProgramState.hpp"
#include "runnables/Runnable.hpp"

class DjiRunnable : public Runnable {
private:
	Vehicle* vehicle;
public:
	DjiRunnable(ProgramState *pState);

	~DjiRunnable();

	void run();
};

#endif /* DJIRUNNABLE_HPP_ */

#ifndef KILLRUNNABLE_HPP_
#define KILLRUNNABLE_HPP_

#include "ProgramState.hpp"
#include "runnables/Runnable.hpp"

class KillRunnable : public Runnable
{
public:
	KillRunnable(ProgramState* pState);
	void run();

	void resetYaw();

private:
	ProgramState *state;
//	uint32_t fileNumber;
//	char fileName[15];
	float getYawOffset();
};

#endif /* KILLRUNNABLE_HPP_ */

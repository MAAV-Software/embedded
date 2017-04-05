#ifndef DATALINKRUNNABLE_HPP_
#define DATALINKRUNNABLE_HPP_

#include <stdint.h>
#include "Runnable.hpp"
#include "ProgramState.hpp"
#include "messaging/emergency_t.h"
#include "messaging/feedback_t.h"

class DataLinkRunnable : public Runnable
{
public:
	DataLinkRunnable(ProgramState *pState);
	void run();

private:
	ProgramState *ps;
};

#endif

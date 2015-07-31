#ifndef DATALINKRUNNABLE_HPP_
#define DATALINKRUNNABLE_HPP_

#include <stdint.h>
#include "Runnable.hpp"
#include "ProgramState.hpp"

class DataLinkRunnable : public Runnable
{
public:
	DataLinkRunnable(ProgramState *pState);
	void run();

private:
	ProgramState *ps;
};

#endif

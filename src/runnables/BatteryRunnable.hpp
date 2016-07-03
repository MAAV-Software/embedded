/**
 * @brief
 * @author Zhengjie
 * @date Jul 30, 2015
 */

#ifndef BATTERYRUNNABLE_HPP_
#define BATTERYRUNNABLE_HPP_

#include "ProgramState.hpp"
#include "runnables/Runnable.hpp"

class BatteryRunnable : public Runnable
{
public:
	BatteryRunnable(ProgramState *pState);
	void run();

private:
	ProgramState *state;
};

#endif /* BATTERYRUNNABLE_HPP_ */

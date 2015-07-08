#ifndef IMURUNNABLE_HPP_
#define IMURUNNABLE_HPP_

#include "ProgramState.hpp"
#include "runnables/Runnable.hpp"

class ImuRunnable : public Runnable
{
private:
	Imu 	*imu;

public:
	ImuRunnable(ProgramState *pState);

	~ImuRunnable();

	void run();

};

#endif /* IMURUNNABLE_HPP_ */

#ifndef I2CRUNNABLE_HPP_
#define I2CRUNNABLE_HPP_

#include "ProgramState.hpp"
#include "runnables/Runnable.hpp"

class I2CRunnable : public Runnable
{
public:
	I2CRunnable(ProgramState *pState);
	void run();

private:
	Px4 	*px4;
	Lidar	*lidar;
};

#endif /* I2CRUNNABLE_HPP_ */

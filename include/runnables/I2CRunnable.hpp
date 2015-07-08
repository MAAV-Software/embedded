#ifndef I2CRUNNABLE_HPP_
#define I2CRUNNABLE_HPP_

#include "ProgramState.hpp"
#include "runnables/Runnable.hpp"

class I2CRunnable : public Runnable
{
private:
	Px4 	*px4;
	Lidar	*lidar;

public:
	I2CRunnable(ProgramState *pState);

	~I2CRunnable();

	void run();

};

#endif /* I2CRUNNABLE_HPP_ */

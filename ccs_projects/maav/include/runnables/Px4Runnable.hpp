#ifndef PX4RUNNABLE_HPP_
#define PX4RUNNABLE_HPP_

#include "VehicleState.hpp"
#include "Runnable.hpp"

class Px4Runnable : public Runnable
{
private:
	VehicleState* _state;

public:
	Px4Runnable(VehicleState* state);

	~Px4Runnable();

	void run();
};

#endif /* PX4RUNNABLE_HPP_ */

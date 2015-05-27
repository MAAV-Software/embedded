#ifndef PX4RUNNABLE_HPP_
#define PX4RUNNABLE_HPP_

#include "../Vehicle.hpp"
#include "Runnable.hpp"

class Px4Runnable : public Runnable
{
private:
	Vehicle* vehicle;

public:
	Px4Runnable(Vehicle* v);

	~Px4Runnable();

	void run();
};

#endif /* PX4RUNNABLE_HPP_ */

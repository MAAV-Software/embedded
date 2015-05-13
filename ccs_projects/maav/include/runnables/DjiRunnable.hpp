#ifndef DJIRUNNABLE_HPP_
#define DJIRUNNABLE_HPP_

#include "../Vehicle.hpp"
#include "runnables/Runnable.hpp"

class DjiRunnable : public Runnable {
private:
	VehicleState* _state;
public:
	DjiRunnable(VehicleState* state);

	~DjiRunnable();

	void run();
};

#endif /* DJIRUNNABLE_HPP_ */

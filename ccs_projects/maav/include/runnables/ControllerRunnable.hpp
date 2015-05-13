#ifndef CONTROLLERRUNNABLE_HPP_
#define CONTROLLERRUNNABLE_HPP_

#include "../Vehicle.hpp"
#include "Runnable.hpp"

class ControllerRunnable : public Runnable {
private:
	VehicleState* _state;

public:
	ControllerRunnable(VehicleState* state);

	~ControllerRunnable();

	void run();
};

#endif /* CONTROLLERRUNNABLE_HPP_ */

#ifndef CONTROLLERRUNNABLE_HPP_
#define CONTROLLERRUNNABLE_HPP_

#include "Runnable.hpp"
#include "VehicleState.hpp"

class ControllerRunnable : public Runnable {
private:
	VehicleState* _state;

public:
	ControllerRunnable(VehicleState* state);

	~ControllerRunnable();

	void run();
};

#endif /* CONTROLLERRUNNABLE_HPP_ */

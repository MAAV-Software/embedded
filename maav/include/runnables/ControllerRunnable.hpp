#ifndef CONTROLLERRUNNABLE_HPP_
#define CONTROLLERRUNNABLE_HPP_

#include "../Vehicle.hpp"
#include "Runnable.hpp"

class ControllerRunnable : public Runnable {
private:
	Vehicle* vehicle;

public:
	ControllerRunnable(Vehicle* v);

	~ControllerRunnable();

	void run();
};

#endif /* CONTROLLERRUNNABLE_HPP_ */

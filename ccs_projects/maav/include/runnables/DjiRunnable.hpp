#ifndef DJIRUNNABLE_HPP_
#define DJIRUNNABLE_HPP_

#include "runnables/Runnable.hpp"
#include "VehicleState.hpp"

class DjiRunnable : public Runnable {
private:
	VehicleState* _state;
public:
	DjiRunnable(VehicleState* state);

	~DjiRunnable();

	void run();
};

#endif /* DJIRUNNABLE_HPP_ */

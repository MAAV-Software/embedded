#ifndef FLIGHTMODERUNNABLE_HPP_
#define FLIGHTMODERUNNABLE_HPP_

#include "../Vehicle.hpp"
#include "runnables/Runnable.hpp"

class FlightModeRunnable : public Runnable {
private:
	// pointer to vehicle state, it is probably shared
	// between multiple classes
	VehicleState* _state;

public:
	FlightModeRunnable(VehicleState* state);

	~FlightModeRunnable();

	void run();
};

#endif /* FLIGHTMODERUNNABLE_HPP_ */

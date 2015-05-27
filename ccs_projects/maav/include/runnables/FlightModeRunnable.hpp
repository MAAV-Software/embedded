#ifndef FLIGHTMODERUNNABLE_HPP_
#define FLIGHTMODERUNNABLE_HPP_

#include "../Vehicle.hpp"
#include "runnables/Runnable.hpp"

class FlightModeRunnable : public Runnable {
private:
	// pointer to vehicle state, it is probably shared
	// between multiple classes
	Vehicle* vehicle;

public:
	FlightModeRunnable(Vehicle* v);

	~FlightModeRunnable();

	void run();
};

#endif /* FLIGHTMODERUNNABLE_HPP_ */

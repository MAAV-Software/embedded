#ifndef DJIRUNNABLE_HPP_
#define DJIRUNNABLE_HPP_

#include "../Vehicle.hpp"
#include "runnables/Runnable.hpp"

class DjiRunnable : public Runnable {
private:
	Vehicle* vehicle;
public:
	DjiRunnable(Vehicle* v);

	~DjiRunnable();

	void run();
};

#endif /* DJIRUNNABLE_HPP_ */

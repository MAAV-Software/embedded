#include "Runabble.hpp"
#include "switch.h"

class SwitchUpdateRunnable : public Runnable {
private:
	SwitchData_t *sw;
public:
	void run();
	
	SwitchUpdateRunnable(SwitchData_t *sw);
};

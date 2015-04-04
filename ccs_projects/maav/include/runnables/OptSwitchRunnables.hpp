#include "Runnable.hpp"
#include "switch.h"

class SwitchUpdateRunnable : public Runnable {
private:
	SwitchData_t *_sw;
public:
	void run();
	
	SwitchUpdateRunnable(SwitchData_t *sw);

	~SwitchUpdateRunnable();
};

// Battery_Monitor.h
// This file declares the Battery_Monitor class.

// See page 124 of Lab 5: ADC of the Getting Started with the Tiva TM4C123G LaunchPad Workshop
// for a list of the include files below:
#ifndef BATTERY_HPP_
#define BATTERY_HPP_

#include <stdint.h>
#include <cstdlib>
#include <cstdio>

#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c123gh6pm.h"

#include "driverlib/gpio.h"
#include "driverlib/fpu.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

class Battery
{
public:
	Battery(const float initVolts = 14.8f, const float minThresh = 3.25f, const uint8_t cellCount = 4);
	void update(const uint32_t rawCode);
	float getVolts() const;
	bool isLow() const;

private:
	float volts;
	float threshold;
	uint8_t numCells;
	float scaleFactor;
};

/*
#define NUM_CELLS 4
#define SHUTDOWN_THRESH 3.2
#define BATT_SAMPLES 5
#define ADC_SCALE 1

class Battery_Monitor
{
	struct battery_data
	    {
	    	double batt_voltage;
	    	double shutdown_thresh;
	    	int num_cells;
	    };

public:
	Battery_Monitor(); // Constructor.
	~Battery_Monitor(); // "Destructor."
	void calc_batt_volt(battery_data*); // Multiply the ADC averaged sample by a scaling factor to
    // obtain the estimated battery voltage.
	double get_batt_volt(battery_data*); // Obtain the estimated battery voltage from a battery_data struct.
	bool low_battery_check(battery_data*); // Deterimine whether the battery voltage is too low.

private:
    battery_data batt_data; // Each instance of the Battery_Monitor class has a private battery_data struct.
};
*/

#endif /* Battery.hpp */

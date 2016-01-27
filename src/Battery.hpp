// Battery_Monitor.h
// This file declares the Battery_Monitor class.

// See page 124 of Lab 5: ADC of the Getting Started with the Tiva TM4C123G LaunchPad Workshop
// for a list of the include files below:

/**
 * @brief Information about the Battery
 *
 * @details Things like voltage, minimum voltage, cell count, ect.
 */
#ifndef BATTERY_HPP_
#define BATTERY_HPP_

#include <stdint.h>
#include <cstdlib>

class Battery
{
public:
	Battery(const float initVolts = 14.8f, const float minThresh = 3.25f, const uint8_t cellCount = 4);
	/**
	 * @brief test3
	 * @details some more details about this here update functions
	 * @param rawCode the raw adc value for battery voltage
	 *
	 */
	void update(const uint32_t rawCode);
	float getVolts() const;
	bool isLow() const;

private:
	float volts; /**< @brief voltage of the battery*/
	float threshold; /**<  @brief minimum acceptable voltage*/
	uint8_t numCells; /**< @brief number of cells in our battery*/
	float scaleFactor; /**< @brief UNKNOWN */
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

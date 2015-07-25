// Battery_Monitor.cpp
// This file defines the Battery_Monitor class.

#include "Battery.hpp"

using namespace std;

Battery::Battery(const float initVolts, const float minThresh,
				 const uint8_t cellCount)
{
	volts = initVolts;
	threshold = minThresh;
	numCells = cellCount;
}

float Battery::getVolts() const
{
	return volts;
}

bool Battery::isLow() const
{
	return ((volts / (float)numCells) <= threshold) ? true : false;
}

void Battery::update(const uint32_t rawCode, const float scaleFactor)
{
	volts = (float)rawCode * scaleFactor;
}

/*
Battery_Monitor::Battery_Monitor() // Constructor.
{
	batt_data.batt_voltage = 0;
	batt_data.shutdown_thresh = SHUTDOWN_THRESH;
	batt_data.num_cells = NUM_CELLS;
}

Battery_Monitor::~Battery_Monitor() // "Destructor."
{
	batt_data.batt_voltage = 0;
	batt_data.shutdown_thresh = 0;
	batt_data.num_cells = 0;
}

void Battery_Monitor::calc_batt_volt(battery_data* batt_data, double battVolt_ADC) // Multiply the ADC averaged sample by a scaling factor to
// obtain the estimated battery voltage.
{
    double ADC_Scale = ADC_SCALE;
	batt_data->batt_voltage = battVolt_ADC * ADC_Scale;
	return;
}

double Battery_Monitor::get_batt_volt(battery_data* batt_data) // Obtain the estimated battery voltage from a battery_data struct.
{
	return batt_data->batt_voltage;
}

bool Battery_Monitor::low_battery_check(battery_data* batt_data) // Deterimine whether the battery voltage is too low.
{
	if (batt_data->batt_voltage <= batt_data->shutdown_thresh)
	{
		return true;
	}
	else
	{
		return false;
	}
}
*/


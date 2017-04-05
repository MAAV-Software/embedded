#ifndef VEHICLE_HPP_
#define VEHICLE_HPP_

#include "Dji.hpp"
#include "FlightMode.hpp"
#include "CtrlLogs.hpp"

// Defines for array sizes
#define NUM_DOFS	4
#define NUM_ANGLES	2

// Error codes for servoIn being out of bounds
#define DJI_SERVOIN_FZ_OOB 0x01;
#define DJI_SERVOIN_XD_OOB 0x02;
#define DJI_SERVOIN_YD_OOB 0x04;
#define DJI_SERVOIN_YAWD_OOB 0x08;

class Vehicle
{
public:
	// Public Methods
	// Constructors
	//Vehicle(){};
	
	void setCtrlInput(float roll, float pitch, float yaw, float thrust);
	
	// returns the DJI values needed to send to it
	Dji getDjiVals() const;

	// Gets RCInputError which has flags set as per DJI_SERVOIN_XXX_OOB
	uint8_t getRCInputError();
	// Sets one or more RCInputError flags (|= with whatever already set)
	void setRCInputError(uint8_t flag);
	// Clears all RCInputError flags
	void clearRCInputError();

private:
	// Controller specific Members
	Dji dji;					// DJI struct of values
	float mass;					// Mass of the Vehicle
	
    uint8_t inputerror; //flags for bad input error

	// Updates the states of the DOFS
	//void setDofStates(const float state[NUM_DOFS][NUM_DOF_STATES]);
};

#endif /* VEHICLE_HPP_ */

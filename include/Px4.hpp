#ifndef PX4_HPP_
#define PX4_HPP_

#include <stdint.h>
#include "Px4Defines.hpp"

// Union for raw array and px4 message frame
typedef union _px4FrameBuffer
{
	px4Frame data;
	uint8_t raw[sizeof(px4Frame)];
} px4FrameBuffer;


// Class representation of PX4
class Px4
{
public:
	Px4();
	
	// parsing function
	void parse(uint8_t *raw);

	// getters for data that also apply appropriate scaling
	uint16_t getFrameCount() const;		// returns the i2c framecount
	uint32_t getSonarTimestamp() const;	// returns timestamp in usec
	uint8_t getGyroRange() const; 		// scaled appropraitely
	float getPixelXFlow() const;		// in pixels/s
   	float getPixelYFlow() const;		// in pixels/s
	float getXFlow() const;				// in m/s
	float getYFlow() const;				// in m/s
	float getQual() const;				// a percentage of how good flow is
	float getGyroXRate() const;			// in rad/s
	float getGyroYRate() const;			// in rad/s
	float getGyroZRate() const;			// in rad/s
	float getZDist() const;				// ground distance in meters

private:
	px4FrameBuffer fb; // frame buffer of PX4 message that is being parsed
};

#endif /* PX4_HPP_ */

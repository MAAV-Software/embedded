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
	void parse(const uint8_t *raw);

	// getters for data that also apply appropriate scaling
	uint16_t getFrameCount() const;		// returns the i2c framecount
	uint32_t getSonarTimestamp();		// returns timestamp in usec
	uint8_t getGyroRange() const; 		// scaled appropraitely
	float getPixelXFlow();				// in pixels/s
   	float getPixelYFlow();				// in pixels/s
	float getXFlow();					// in m/s
	float getYFlow();					// in m/s
	float getQual();					// a percentage of how good flow is
	float getGyroXRate();				// in rad/s
	float getGyroYRate();				// in rad/s
	float getGyroZRate();				// in rad/s
	float getZDist();					// ground distance in meters
	float getTimestamp() const;
	void RecordTime(float time);
private:
	px4FrameBuffer fb; // frame buffer of PX4 message that is being parsed
	float timestamp;   // timestamp when parsing
};

#endif /* PX4_HPP_ */

#ifndef PX4_HPP_
#define PX4_HPP_

#include <stdint.h>

class Px4 {
private:
	uint16_t _frameCount;
	int16_t _pixelFlowXSum; // accumulated x flow in pixels * 10 since last I2C frame
	int16_t _pixelFlowYSum; // accumulated y flow in pixels * 10 since last I2C frame
	int16_t _flowCompMX; // x velocity * 1000 in meters / timestep
	int16_t _flowCopmMY; // y velocity * 1000 in meters / timestep
	int16_t _qual; // optical flow quality / confidence 0: bad, 255: maximum quality
	int16_t _gyroXRate;
	int16_t _gyroYRate;
	int16_t _gyroZRate;
	uint8_t _gyroRange;
	uint8_t _sonarTimestamp;
	int16_t _groundDistance;
public:
	Px4();

	~Px4();

	uint16_t frameCount() const;
};

#endif /* PX4_HPP_ */

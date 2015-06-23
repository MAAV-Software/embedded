/*
 * Lidar.hpp
 *
 *  Created on: May 21, 2015
 *      Author: James Connolly
 */

#ifndef LIDAR_HPP_
#define LIDAR_HPP_

#include <stdint.h>

class Lidar 
{
public:
	Lidar();
	
	// parses based on size
	void parse(uint8_t *raw, const uint8_t size);
	
	// getters for distance and velocity
	float getDist() const;
	float getVel() const;

private:
	float dist; // distance measurement in meters
	float vel;	// velocity measurement in m/s
};

#endif /* LIDAR_HPP_ */

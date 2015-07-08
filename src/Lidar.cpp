/*
 * Lidar.cpp
 *
 *  Created on: May 21, 2015
 *      Author: James Connolly
 */

#include "Lidar.hpp"
//#include "LidarDefines.hpp"

Lidar::Lidar() 
{
	dist = 0;
	vel = 0;
}

float Lidar::getDist() const 
{
	return dist;
}

float Lidar::getVel() const
{
	return vel;
}

void Lidar::parse(uint8_t* raw, const uint8_t size) 
{
	switch (size)
	{
		case LIDAR_DIST_SIZE:
			// raw array is [dist upper byte, dist lower byte], 
			// so we join them together and scale them into meters 
			// since the raw value is in cm
			dist =  (float)(((uint32_t)raw[0] << 8) | raw[1]) / 100.0;
			break;
		case LIDAR_VEL_SIZE:
			// velocity is just an 8-bit singed int with no scaling
			vel = (float)((int8_t)(*raw));
			break;
		default:
			break;
	}
}

/*
 * Lidar.cpp
 *
 *  Created on: Jun 24, 2015
 *      Author: Zhengjie
 */

#include "Lidar.hpp"

Lidar::Lidar()
{
	for (int i = 0; i < LIDAR_DATA_LENGTH; i++)
	{
		lidar_buffer[i] = 0;
		raw_in[i] = 0;
	}

}

void Lidar::parse(void)
{
	std::memcpy(lidar_buffer, raw_in, LIDAR_DATA_LENGTH);
}

uint32_t Lidar::getDistance(void)
{
	return (lidar_buffer[0] << 8) | lidar_buffer[1];
}

uint8_t* Lidar::get_raw_in(void)
{
	return raw_in;
}

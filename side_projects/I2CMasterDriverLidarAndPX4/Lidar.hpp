/*
 * Lidar.hpp
 *
 *  Created on: Jun 24, 2015
 *      Author: Zhengjie
 */

#ifndef LIDAR_HPP_
#define LIDAR_HPP_

#include "I2C_defines.hpp"

class Lidar
{
public:
	Lidar();
	void parse(void);
	uint32_t getDistance(void);
	uint8_t * get_raw_in(void);

private:
	uint8_t lidar_buffer[LIDAR_DATA_LENGTH];
	uint8_t raw_in[LIDAR_DATA_LENGTH];

};


#endif /* LIDAR_HPP_ */

/*
 * Lidar.h
 *
 *  Created on: May 21, 2015
 *      Author: James Connolly
 */

#ifndef SRC_LIDAR_H_
#define SRC_LIDAR_H_
#include "stdint.h"

class Lidar {
public:
	Lidar();
	virtual ~Lidar();
	void parse(uint8_t *raw);
	float getHeight() const;
private:
	volatile union{
		uint8_t internal[sizeof(float) / sizeof(uint8_t)];
		float height;
	}data;
};

#endif /* SRC_LIDAR_H_ */

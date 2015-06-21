/*
 * Lidar.cpp
 *
 *  Created on: May 21, 2015
 *      Author: James Connolly
 */

#include "Lidar.h"

Lidar::Lidar() {
	// TODO Auto-generated constructor stub

}

Lidar::~Lidar() {
	// TODO Auto-generated destructor stub
}

float Lidar::getHeight() const {
	return data.height;
}

void Lidar::parse(uint8_t * raw) {
	for(int i = 0; i < sizeof(float) / sizeof(uint8_t); ++i){
		(data.internal)[i] = *raw;
		++raw;
	}
}

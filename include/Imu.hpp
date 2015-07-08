/*
 * imu_uart.h
 *
 *  Created on: Feb 21, 2015
 *      Authors: Carl Chan, Zhengjie Cui
 */

#ifndef IMU_HPP_
#define IMU_HPP_

#include "Imu_Defines.hpp"

class Imu {
public:
	Imu();
	void parse(const uint8_t * data);
	// Return data
	float getAccX() const;
	float getAccY() const;
	float getAccZ() const;
	float getRoll() const;
	float getPitch() const;
	float getYaw() const;
	float getAngRateX() const;
	float getAngRateY() const;
	float getAngRateZ() const;
	float getMagX() const;
	float getMagY() const;
	float getMagZ() const;
	uint32_t getTimer() const;

private:
	float AccX;
	float AccY;
	float AccZ;
	float AngRateX;
	float AngRateY;
	float AngRateZ;
	float MagX;
	float MagY;
	float MagZ;
	float M[NUM_M_VAL];
	uint32_t Timer;
};

uint32_t Bytes2Int(const unsigned char *raw, unsigned int i);

float Bytes2Float(const unsigned char *raw, unsigned int i);

#endif /* IMU_UART_H_ */

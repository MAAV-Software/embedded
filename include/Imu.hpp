/*
 * imu_uart.h
 *
 *  Created on: Feb 21, 2015
 *      Authors: Carl Chan, Zhengjie Cui
 */

#ifndef IMU_HPP_
#define IMU_HPP_

#include <stdint.h>
#include "ImuDefines.hpp"

class Imu
{
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
	void getRotMat(float dest[NUM_M_VAL]);

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

	/*
	* Here are the corresponding M rotation matrix entries and their indecies
	* M[0 1 2;
	*   3 4 5;
	*   6 7 8]
	*/
};

uint32_t Bytes2Int(const uint8_t *raw, const unsigned int i);
float Bytes2Float(const uint8_t *raw, const unsigned int i);

#endif /* IMU_UART_H_ */

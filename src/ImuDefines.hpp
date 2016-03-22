/*
 * ImuDefines.hpp
 *
 *  Created on: May 28, 2015
 *      Author: zjcui
 */
#ifndef IMUDEFINES_HPP_
#define IMUDEFINES_HPP_

#include <stdint.h>

#define NUM_M_VAL 9
#define MAX_IMU_DATA_LENGTH 79

#define MEASUREMENT_CMD 0xCC
#define ACCEL_CALIB_CMD 0xC9
#define GYRO_CALIB_CMD 0xCD

namespace MaavImu 
{

extern const uint32_t MEASUREMENT_DATA_LENGTH;
extern const uint32_t ACCEL_BIAS_DATA_LENGTH;
extern const uint32_t GYRO_BIAS_DATA_LENGTH;

}

#endif /* IMU_DEFINES_HPP_ */

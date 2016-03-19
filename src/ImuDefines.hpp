/*
 * ImuDefines.hpp
 *
 *  Created on: May 28, 2015
 *      Author: zjcui
 */
namespace MaavImu{

#ifndef IMUDEFINES_HPP_
#define IMUDEFINES_HPP_

#define NUM_M_VAL 9
#define IMU_DATA_LENGTH 79
#define IMU_ACC_CALIBRATE_DATA_LENGTH 19
#define IMU_GYRO_CALIBRATE_DATA_LENGTH 19

uint32_t MeasurementDataLength = 79;
uint32_t AccelBiasDataLength = 19;
uint32_t GyroBiasDataLength = 19;

uint8_t ImuMeasurementCommand = 0xCC;
uint8_t AccCalibCommand = 0xC9;
uint8_t GyroCalibCommand = 0xCD;




#endif /* IMU_DEFINES_HPP_ */

}

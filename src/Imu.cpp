/*
 * imu_uart.c
 *
 *  Created on: Feb 21, 2015
 *      Authors: Carl Chan Zhengjie Cui
 */

#include <stdint.h>
#include <cmath>

#include "Imu.hpp"
#include "ImuDefines.hpp"
#include "time_util.h"
#include "MaavMath.hpp"

using namespace std;
using namespace MaavMath;
using namespace MaavImu;

typedef union u16union
{
    uint16_t number;
    uint8_t buf[sizeof(uint16_t)];
} BytesU16;

typedef union u32union
{
    uint32_t number;
    uint8_t buf[sizeof(uint32_t)];
} BytesU32;

typedef union f32union
{
    float number;
    uint8_t buf[sizeof(float)];
} BytesF32;


Imu::Imu()
{
	dataStruct.refYaw = 0;
	dataStruct.AccX = 0;
	dataStruct.AccY = 0;
	dataStruct.AccZ = 0;
	dataStruct.AngRateX = 0;
	dataStruct.AngRateY = 0;
	dataStruct.AngRateZ = 0;
	dataStruct.MagX = 0;
	dataStruct.MagY = 0;
	dataStruct.MagZ = 0;
	dataStruct.GyroBiasX = 0;
	dataStruct.GyroBiasY = 0;
	dataStruct.GyroBiasZ = 0;
	dataStruct.AccBiasX = 0;
	dataStruct.AccBiasY = 0;
	dataStruct.AccBiasZ = 0;
	
    for (unsigned int i = 0; i < NUM_M_VAL; ++i) dataStruct.M[i] = 0;
	
    dataStruct.Timer = 1;
}

MicroStrainCmd Imu::formatStopContMode()
{
	MicroStrainCmd m;
	m.length = 3;
	m.buf[0] = 0xFA;
	m.buf[1] = 0x75;
	m.buf[2] = 0xB4;

	return m;
}

MicroStrainCmd Imu::formatSoftResetCmd()
{
	MicroStrainCmd m;
	m.length = 3;
	m.buf[0] = 0xFE;
	m.buf[1] = 0x9E;
	m.buf[2] = 0x3A;

	return m;
}

MicroStrainCmd Imu::formatMeasCmd()
{
    MicroStrainCmd m;
    m.buf[0] = (uint8_t)MEASUREMENT_CMD;
    m.length = 1;
    
    return m;
}

MicroStrainCmd Imu::formatGyroBiasCmd(const uint16_t samplingTime)
{
    MicroStrainCmd m;
    m.length = 5;
    
    m.buf[0] = (uint8_t)GYRO_CALIB_CMD;
    m.buf[1] = 0xC1;
    m.buf[2] = 0x29;
    u16ToBytes(m.buf, 3, samplingTime);
     
    return m;
}

MicroStrainCmd Imu::formatAccelBiasCmd(const float accXBias,
                                       const float accYBias,
                                       const float accZBias)
{
    MicroStrainCmd m;
    m.length = 15;
    
    m.buf[0] = (uint8_t)ACCEL_CALIB_CMD;
    m.buf[1] = 0xB7;
    m.buf[2] = 0x44;
    floatToBytes(m.buf, 3, accXBias);
    floatToBytes(m.buf, 7, accYBias); 
    floatToBytes(m.buf, 11, accZBias); 
    
    return m;
}
                                

bool Imu::goodChecksum(const uint8_t* data, uint32_t size)
{
    int16_t checksum = 0;
    for (uint32_t i = 0; i < size - 2; ++i) checksum += data[i];

    int16_t responseChecksum = ((uint16_t)data[size - 2] << 8) | data[size - 1];
    
    return checksum == responseChecksum;
}

void Imu::parseMeasurements(const uint8_t* data)
{
    if (!goodChecksum(data, MEASUREMENT_DATA_LENGTH)) return; // check checksum

    dataStruct.gAccX = Bytes2Float(data, 1);
    dataStruct.gAccY = Bytes2Float(data, 5);
    dataStruct.gAccZ = Bytes2Float(data, 9);

    dataStruct.AccX = Gravity * getgAccX();
    dataStruct.AccY = Gravity * getgAccY();
    dataStruct.AccZ = Gravity * getgAccZ();
    dataStruct.AngRateX = Bytes2Float(data, 13);
    dataStruct.AngRateY = Bytes2Float(data, 17);
    dataStruct.AngRateZ = Bytes2Float(data, 21);
    dataStruct.MagX = Bytes2Float(data, 25);
    dataStruct.MagY = Bytes2Float(data, 29);
    dataStruct.MagZ = Bytes2Float(data, 33);

    for (unsigned int i = 0; i < NUM_M_VAL; ++i)
    	dataStruct.M[i] = Bytes2Float(data, (37 + (i * 4)));
    /*
    * Here are the corresponding M rotation matrix entries and their indecies
    * M[0 1 2;
    *   3 4 5;
    *   6 7 8]
    */

    dataStruct.Timer = Bytes2Int(data, 73);
}

void Imu::parseAccCal(const uint8_t* data)
{
    if (!goodChecksum(data, ACCEL_BIAS_DATA_LENGTH)) return; // check checksum

    dataStruct.AccBiasX = Bytes2Float(data, 1);
    dataStruct.AccBiasY = Bytes2Float(data, 5);
    dataStruct.AccBiasZ = Bytes2Float(data, 9);
    dataStruct.Timer = Bytes2Int(data, 13);
}

void Imu::parseGyroBias(const uint8_t* data)
{
    if (!goodChecksum(data, GYRO_BIAS_DATA_LENGTH)) return; // check checksum

    dataStruct.GyroBiasX = Bytes2Float(data, 1);
    dataStruct.GyroBiasY = Bytes2Float(data, 5);
    dataStruct.GyroBiasZ = Bytes2Float(data, 9);
    dataStruct.Timer = Bytes2Int(data, 13);
}

void Imu::parse(const uint8_t* data)
{
    // Switch on command response (first byte of data array)
	switch (data[0])
	{
		case MEASUREMENT_CMD : parseMeasurements(data); break;
		case ACCEL_CALIB_CMD : parseAccCal(data);       break;
		case GYRO_CALIB_CMD  : parseGyroBias(data);     break;
        default: break;
	}
}

imu_t* Imu::getImuData()
{
	return &dataStruct;
}

void Imu::getRotMat(float dest[NUM_M_VAL])
{
	for (int i = 0; i < NUM_M_VAL; ++i) dest[i] = dataStruct.M[i];
}

const float* Imu::getRotMat() const 
{
	return dataStruct.M;
}

// Return data
float Imu::getgAccX() const
{
	return dataStruct.gAccX;
}

float Imu::getgAccY() const
{
	return dataStruct.gAccY;
}

float Imu::getgAccZ() const
{
	return dataStruct.gAccZ;
}

float Imu::getAccX() const
{
	return dataStruct.AccX;
}

float Imu::getAccY() const
{
	return dataStruct.AccY;
}

float Imu::getAccZ() const
{
	return dataStruct.AccZ;
}

float Imu::getRoll() const
{
	return atan2(dataStruct.M[5], dataStruct.M[8]);
}

float Imu::getPitch() const
{
	return asin(-dataStruct.M[2]);
}

float Imu::getYaw() const
{
	return atan2(dataStruct.M[1], dataStruct.M[0]) - dataStruct.refYaw;
}

float Imu::getAngRateX() const
{
	return dataStruct.AngRateX;
}

float Imu::getAngRateY() const
{
	return dataStruct.AngRateY;
}

float Imu::getAngRateZ() const
{
	return dataStruct.AngRateZ;
}

float Imu::getMagX() const
{
	return dataStruct.MagX;
}

float Imu::getMagY() const
{
	return dataStruct.MagY;
}

float Imu::getMagZ() const
{
	return dataStruct.MagZ;
}

int32_t Imu::getTimer() const
{
	return dataStruct.Timer / 62500;
}

float Imu::getTimestamp() const
{
	return dataStruct.timestamp;
}

void Imu::RecordTime(float time)
{
	dataStruct.timestamp = time;
}

void Imu::setRefYaw(float newRefYaw)
{
	dataStruct.refYaw = newRefYaw;
}

float Imu::getRefYaw() const 
{
    return dataStruct.refYaw;
}

float Imu::getAccBiasX() const
{
  return dataStruct.AccBiasX;
}

float Imu::getAccBiasY() const
{
	return dataStruct.AccBiasY;
}

float Imu::getAccBiasZ() const
{
	return dataStruct.AccBiasZ;
}

float Imu::getGyroBiasX() const
{
	return dataStruct.GyroBiasX;
}

float Imu::getGyroBiasY() const 
{
	return dataStruct.GyroBiasY;
}

float Imu::getGyroBiasZ() const
{
	return dataStruct.GyroBiasZ;
}

void floatToBytes(uint8_t* dest, uint32_t idx, float num)
{
    BytesF32 data;
    data.number = num;
    
    dest[idx]     = data.buf[3];
    dest[idx + 1] = data.buf[2];
    dest[idx + 2] = data.buf[1];
    dest[idx + 3] = data.buf[0];
}

void u16ToBytes(uint8_t* dest, uint32_t idx, uint16_t num)
{
    BytesU16 data;
    data.number = num;

    dest[idx]     = data.buf[1];
    dest[idx + 1] = data.buf[0];
}

uint32_t Bytes2Int(const uint8_t *raw, const unsigned int i)
{
    BytesU32 data;

	data.buf[0] = raw[i + 3];
	data.buf[1] = raw[i + 2];
	data.buf[2] = raw[i + 1];
	data.buf[3] = raw[i + 0];

	return data.number;
}

float Bytes2Float(const uint8_t *raw, const unsigned int i)
{
    BytesF32 data;

	data.buf[0] = raw[i + 3];
	data.buf[1] = raw[i + 2];
	data.buf[2] = raw[i + 1];
	data.buf[3] = raw[i + 0];

	return data.number;
}

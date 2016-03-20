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
	refYaw = 0;
	AccX = 0;
	AccY = 0;
	AccZ = 0;
	AngRateX = 0;
	AngRateY = 0;
	AngRateZ = 0;
	MagX = 0;
	MagY = 0;
	MagZ = 0;
	
    for (unsigned int i = 0; i < NUM_M_VAL; ++i) M[i] = 0;
	
    Timer = 1;
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

    AccX = Gravity * Bytes2Float(data, 1);
    AccY = Gravity * Bytes2Float(data, 5);
    AccZ = Gravity * Bytes2Float(data, 9);
    AngRateX = Bytes2Float(data, 13);
    AngRateY = Bytes2Float(data, 17);
    AngRateZ = Bytes2Float(data, 21);
    MagX = Bytes2Float(data, 25);
    MagY = Bytes2Float(data, 29);
    MagZ = Bytes2Float(data, 33);

    for (unsigned int i = 0; i < NUM_M_VAL; ++i)
        M[i] = Bytes2Float(data, (37 + (i * 4)));
    /*
    * Here are the corresponding M rotation matrix entries and their indecies
    * M[0 1 2;
    *   3 4 5;
    *   6 7 8]
    */

    Timer = Bytes2Int(data, 73);
}

void Imu::parseAccCal(const uint8_t* data)
{
    if (!goodChecksum(data, ACCEL_BIAS_DATA_LENGTH)) return; // check checksum

	AccBiasX = Bytes2Float(data, 1);
	AccBiasY = Bytes2Float(data, 5);
    AccBiasZ = Bytes2Float(data, 9);
    Timer = Bytes2Int(data, 13);
}

void Imu::parseGyroBias(const uint8_t* data)
{
    if (!goodChecksum(data, GYRO_BIAS_DATA_LENGTH)) return; // check checksum

	GyroBiasX = Bytes2Float(data, 1);
	GyroBiasY = Bytes2Float(data, 5);
    GyroBiasZ = Bytes2Float(data, 9);
    Timer = Bytes2Int(data, 13);
}

void Imu::parse(const uint8_t* data)
{
    // Switch on command response (first byte of data array)
	switch (data[0])
	{
		case MEASUREMENT_CMD : parseMeasurements(data);  break;
		case ACCEL_CALIB_CMD : parseAccCal(data);       break;
		case GYRO_CALIB_CMD  : parseGyroBias(data);     break;
        default: break;
	}
}

void Imu::getRotMat(float dest[NUM_M_VAL])
{
	for (int i = 0; i < NUM_M_VAL; ++i) dest[i] = M[i];
}

const float* Imu::getRotMat() const 
{
	return M;
}

// Return data
float Imu::getAccX() const
{
	return AccX;
}

float Imu::getAccY() const
{
	return AccY;
}

float Imu::getAccZ() const
{
	return AccZ;
}

float Imu::getRoll() const
{
	return atan2(M[5], M[8]);
}

float Imu::getPitch() const
{
	return asin(-M[2]);
}

float Imu::getYaw() const
{
	return atan2(M[1], M[0]) - refYaw;
}

float Imu::getAngRateX() const
{
	return AngRateX;
}

float Imu::getAngRateY() const
{
	return AngRateY;
}

float Imu::getAngRateZ() const
{
	return AngRateZ;
}

float Imu::getMagX() const
{
	return MagX;
}

float Imu::getMagY() const
{
	return MagY;
}

float Imu::getMagZ() const
{
	return MagZ;
}

uint32_t Imu::getTimer() const
{
	return Timer / 62500;
}

float Imu::getTimestamp() const
{
	return timestamp;
}

void Imu::RecordTime(float time)
{
	timestamp = time;
}

void Imu::setRefYaw(float newRefYaw)
{
    refYaw = newRefYaw;
}

float Imu::getRefYaw() const 
{
    return refYaw;
}

float Imu::getAccBiasX() const
{
  return AccBiasX;
}

float Imu::getAccBiasY() const
{
	return AccBiasY;
}

float Imu::getAccBiasZ() const
{
	return AccBiasZ;
}

float Imu::getGyroBiasX() const
{
	return GyroBiasX;
}

float Imu::getGyroBiasY() const 
{
	return GyroBiasY;
}

float Imu::getGyroBiasZ() const
{
	return GyroBiasZ;
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

    dest[idx]     = data.buf[3];
    dest[idx + 1] = data.buf[2];
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

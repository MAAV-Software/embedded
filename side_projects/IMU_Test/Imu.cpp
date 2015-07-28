/*
 * imu_uart.c
 *
 *  Created on: Feb 21, 2015
 *      Authors: Carl Chan Zhengjie Cui
 */


#include "Imu.hpp"

Imu::Imu()
{
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
	Timer = 0;
}

void Imu::parse(uint8_t* const data)
{
//	if (IMU_DATA_LENGTH < 79)
//	{
//		//TODO: may want to set an error flag
//		return;
//	}

	// Check checksum
	int16_t tChksum = 0;
	for (unsigned int i = 0; i < 77; ++i) tChksum += data[i];

	int16_t tResponseChksum = 0;
	tResponseChksum = (uint16_t)data[77] << 8;
	tResponseChksum += data[78];

//	uint16_t tmp = ((uint16_t)data[77] << 8) + data[78];

	if (tChksum != tResponseChksum)
//	if (tChksum != (int16_t)tmp)
	{
		return;
	}

	AccX = Bytes2Float(data, 1);
	AccY = Bytes2Float(data, 5);
	AccZ = Bytes2Float(data, 9);
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
//		M[0] = Bytes2Float(data, 37); //M11
//		M[1] = Bytes2Float(data, 41); //M12
//		M[2] = Bytes2Float(data, 45); //M13
//		M[3] = Bytes2Float(data, 49); //M21
//		M[4] = Bytes2Float(data, 53); //M22
//		M[5] = Bytes2Float(data, 57); //M23
//		M[6] = Bytes2Float(data, 61); //M31
//		M[7] = Bytes2Float(data, 65); //M32
//		M[8] = Bytes2Float(data, 69); //M33

	Timer = Bytes2Int(data, 73);

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
	return atan2(M[1], M[0]);
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
	return Timer/62500;
}

uint32_t Bytes2Int(const unsigned char * const raw, unsigned int i)
{
	union B2I{
	   unsigned char buf[4];
	   uint32_t number;
	}data;

	data.buf[0] = raw[i+3];
	data.buf[1] = raw[i+2];
	data.buf[2] = raw[i+1];
	data.buf[3] = raw[i+0];

	return data.number;
}

float Bytes2Float(const unsigned char * const raw, unsigned int i)
{
	union B2F{
	   unsigned char buf[4];
	   float number;
	}data;

	data.buf[0] = raw[i+3];
	data.buf[1] = raw[i+2];
	data.buf[2] = raw[i+1];
	data.buf[3] = raw[i+0];

	return data.number;
}


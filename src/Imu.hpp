/*
 * Imu class
 *
 *  Created on: Feb 21, 2015
 *      Authors: Carl Chan, Zhengjie Cui
 *  Updated July 2015 Zhengjie Cui, Sajan Patel
 *  Updated March 2016 Sajan Patel, Jay Tayade
 */

#ifndef IMU_HPP_
#define IMU_HPP_

#include <stdint.h>
#include "ImuDefines.hpp"
#include "messaging/imu_t.h"

#define MAX_MS_CMD_LENGTH 15

typedef struct MSCmd
{
    uint8_t buf[MAX_MS_CMD_LENGTH];
    uint32_t length;
} MicroStrainCmd;

class Imu
{
public:
	Imu();
	
    void parse(const uint8_t* data);

    MicroStrainCmd formatMeasCmd(); 
    MicroStrainCmd formatAccelBiasCmd(const float accXBias, 
                                      const float accYBias,
                                      const float accZBias);
    MicroStrainCmd formatGyroBiasCmd(const uint16_t samplingTime);
    MicroStrainCmd formatStopContMode();
    MicroStrainCmd formatSoftResetCmd();

    // Return data
    imu_t* getImuData();

	float getAccX() const;
	float getAccY() const;
	float getAccZ() const;

	float getgAccX() const;
	float getgAccY() const;
	float getgAccZ() const;

	float getRoll() const;
	float getPitch() const;
	float getYaw() const;
	float getAngRateX() const;
	float getAngRateY() const;
	float getAngRateZ() const;
	float getMagX() const;
	float getMagY() const;
	float getMagZ() const;
	int32_t getTimer() const;
	void getRotMat(float dest[NUM_M_VAL]);
	
    
    /* RotMat return matrix M.
	* Here are the corresponding M rotation matrix entries and their indecies
	* M[0 1 2;
	*   3 4 5;
	*   6 7 8]
	*/
	const float* getRotMat() const;
	
    float getTimestamp() const;
	void RecordTime(float time);
	void setRefYaw(float newRefYaw);
	float getRefYaw() const;

	float getAccBiasX() const;
	float getAccBiasY() const;
	float getAccBiasZ() const;
	float getGyroBiasX() const;
	float getGyroBiasY() const;
	float getGyroBiasZ() const;

private:

	imu_t dataStruct;
/*
	float refYaw;

	float gAccX;
	float gAccY;
	float gAccZ;

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
	float timestamp;
	uint32_t Timer;
	float GyroBiasX;
	float GyroBiasY;
	float GyroBiasZ;
	float AccBiasX;
	float AccBiasY;
	float AccBiasZ;
*/
    bool goodChecksum(const uint8_t* data, uint32_t size);
    void parseMeasurements(const uint8_t* data);
    void parseAccCal(const uint8_t* data);
    void parseGyroBias(const uint8_t* data);
};

// These all convert between raw bytes and data types and flip endianness
uint32_t Bytes2Int(const uint8_t *raw, const unsigned int i);
float Bytes2Float(const uint8_t *raw, const unsigned int i);
void floatToBytes(uint8_t* dest, uint32_t idx, float num);
void u16ToBytes(uint8_t* dest, uint32_t idx, uint16_t num);

#endif /* IMU_UART_H_ */

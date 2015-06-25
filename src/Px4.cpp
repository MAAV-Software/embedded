#include <cstdlib>
#include <stdint.h>
#include "Px4.hpp"
#include "Px4Defines.hpp"

Px4::Px4()
{
	for (uint32_t i = 0; i < sizeof(px4Frame); ++i) fb.raw[i] = 0;
}

void Px4::parse(const uint8_t *raw)
{
	for (uint32_t i = 0; i < sizeof(px4Frame); ++i) fb.raw[i] = raw[i];
}

uint16_t Px4::getFrameCount() const
{
	return fb.data.frame_count;
}

uint32_t Px4::getSonarTimestamp()
{
	return 1000 * (uint32_t)fb.data.sonar_timestamp;
}

uint8_t Px4::getGyroRange() const
{
	return fb.data.gyro_range;
}

float Px4::getPixelXFlow()
{
	return (float)fb.data.pixel_flow_x_sum / 10.0f;
}

float Px4::getPixelYFlow() 
{
	return (float)fb.data.pixel_flow_y_sum / 10.0f;
}

float Px4::getXFlow()
{
	return (float)fb.data.flow_comp_m_x / 1000.0f;
}

float Px4::getYFlow()
{
	return (float)fb.data.flow_comp_m_y / 1000.0f;
}

float Px4::getQual()
{
	return (float)fb.data.qual / 255.0f;
}

float Px4::getGyroXRate()
{
	return (float)fb.data.gyro_x_rate;
}

float Px4::getGyroYRate()
{
	return (float)fb.data.gyro_y_rate;
}

float Px4::getGyroZRate()
{
	return (float)fb.data.gyro_z_rate;
}

float Px4::getZDist()
{
	return (float)fb.data.ground_distance / 1000.0f;
}

// End of File

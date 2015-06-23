#include <cstdlib>
#include <stdint.h>
#include "Px4.hpp"
#include "Px4Defines.hpp"

Px4::Px4()
{
	for (int i = 0; i < sizeof(px4Frame); ++i) fb.raw[i] = 0;
}

void Px4::Parse(const uint8_t *raw)
{
	for (int i = 0; i < sizeof(px4Frame); ++i) fb.raw[i] = raw[i];
}

uint16_t Px4::getFrameCount() const
{
	return fb.data.frame_count
}

uint32_t Px4::getSonarTimestamp() const
{
	return 1000 * (uint32_t)fb.data.sonar_timestamp;
}

uint8_t Px4::getGyroRange() const
{
	return fb.data.gyro_range;
}

float Px4::getPixelXFlow() const
{
	return (float)fb.data.pixel_flow_x_sum / 10.0;
}

float Px4::getPixelYFlow() const
{
	return (float)fb.data.pixel_flow_y_sum / 10.0;
}

float Px4::getXFlow() const
{
	return (float)fb.data.flow_comp_m_x / 1000.0;
}

float Px4::getYFlow() const
{
	return (float)fb.data.flow_comp_m_y / 1000.0;
}

float Px4::getQual() const
{
	return (float)fb.data.qual / 255.0;
}

float Px4::getGyroXRate() const
{
	return (float)fb.data.gyro_x_rate;
}

float Px4::getGyroYRate() const
{
	return (float)fb.data.gyro_y_rate;
}

float Px4::getGyroZRate() const
{
	return (float)fb.data.gyro_z_rate;
}

float Px4::getZDist() const
{
	return (float)fb.data.ground_dist / 1000.0;
}

// End of File

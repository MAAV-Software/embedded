/*
 * PX4.cpp
 *
 *  Created on: Jun 24, 2015
 *      Author: Zhengjie
 */

#include "PX4.hpp"

PX4::PX4()
{
	for (int i = 0; i < sizeof(PX4_data); i++)
	{
		px4_buffer.raw[i] = 0;
		raw_in[i] = 0;
	}
}

void PX4::parse()
{
	std::memcpy(px4_buffer.raw, raw_in, sizeof(PX4_data));
}

int16_t PX4::getHeight()
{
	return px4_buffer.data.ground_distance;
}

uint16_t PX4::get_frame_count(void)
{
	return px4_buffer.data.frame_count;
}

int16_t PX4::get_pixel_flow_x_sum(void)
{
	return px4_buffer.data.pixel_flow_x_sum;
}

int16_t PX4::get_pixel_flow_y_sum(void)
{
	return px4_buffer.data.pixel_flow_y_sum;
}

int16_t PX4::get_flow_comp_m_x(void)
{
	return px4_buffer.data.flow_comp_m_x;
}

int16_t PX4::get_flow_comp_m_y(void)
{
	return px4_buffer.data.flow_comp_m_y;
}

int16_t PX4::get_qual(void)
{
	return px4_buffer.data.qual;
}

int16_t PX4::get_gyro_x_rate(void)
{
	return px4_buffer.data.gyro_x_rate;
}

int16_t PX4::get_gyro_y_rate(void)
{
	return px4_buffer.data.gyro_y_rate;
}

int16_t PX4::get_gyro_z_rate(void)
{
	return px4_buffer.data.gyro_z_rate;
}

uint8_t PX4::get_gyro_range(void)
{
	return px4_buffer.data.gyro_range;
}

uint8_t PX4::getTimestep(void)
{
	return px4_buffer.data.sonar_timestamp;
}

uint8_t* PX4::get_raw_in(void)
{
	return raw_in;
}

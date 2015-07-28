/*
 * PX4.hpp
 *
 *  Created on: Jun 24, 2015
 *      Author: Zhengjie
 */

#ifndef PX4_HPP_
#define PX4_HPP_

#include "I2C_defines.hpp"

//******************************PX4 Data Struct*********************************
typedef struct _PX4_data
{
	uint16_t frame_count;// counts created I2C frames [#frames]
	int16_t pixel_flow_x_sum;// latest x flow measurement in pixels*10 [pixels]
	int16_t pixel_flow_y_sum;// latest y flow measurement in pixels*10 [pixels]
	int16_t flow_comp_m_x;// x velocity*1000 [millimeters/sec]
	int16_t flow_comp_m_y;// y velocity*1000 [millimeters/sec]
	int16_t qual;// Optical flow quality / confidence [0: bad, 255: maximum quality]
	int16_t gyro_x_rate; // latest gyro x rate [rad/sec]
	int16_t gyro_y_rate; // latest gyro y rate [rad/sec]
	int16_t gyro_z_rate; // latest gyro z rate [rad/sec]
	uint8_t gyro_range; // gyro range [0 .. 7] equals [50 deg/sec .. 2000 deg/sec]
	uint8_t sonar_timestamp;// time since last sonar update [milliseconds]
	int16_t ground_distance;// Ground distance in meters*1000 [millimeters]. Positive value: distance known. Negative value: Unknown distance
} PX4_data;

class PX4
{
public:
	PX4();
	void parse(void);
	int16_t getHeight(void);
	uint8_t getTimestep(void);
	uint16_t get_frame_count(void);
	int16_t  get_pixel_flow_x_sum(void);
	int16_t  get_pixel_flow_y_sum(void);
	int16_t  get_flow_comp_m_x(void);
	int16_t  get_flow_comp_m_y(void);
	int16_t  get_qual(void);
	int16_t  get_gyro_x_rate(void);
	int16_t  get_gyro_y_rate(void);
	int16_t  get_gyro_z_rate(void);
	uint8_t  get_gyro_range(void);
	uint8_t* get_raw_in(void);

private:
	typedef union _PX4_frame
	{
		char raw[sizeof(PX4_data)];
		PX4_data data;
	} PX4_frame;

	PX4_frame px4_buffer;

	// This arrary is used to get the incoming data from I2C bus, and immeditely copy to px4_buffer.raw in parse()
	uint8_t raw_in[sizeof(PX4_data)];
};

#endif /* PX4_HPP_ */

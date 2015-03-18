/*
 * px4_i2c.h
 *
 *  Created on: Jul 7, 2014
 *      Author: Jonathan Kurzer
 */

#ifndef PX4_I2C_H_
#define PX4_I2C_H_

#define PX4_ADDRESS		0x43

typedef struct _PX4_data
{
	uint16_t frame_count;// counts created I2C frames
	int16_t pixel_flow_x_sum;// accumulated x flow in pixels*10 since last I2C frame
	int16_t pixel_flow_y_sum;// accumulated y flow in pixels*10 since last I2C frame
	int16_t flow_comp_m_x;// x velocity*1000 in meters / timestep
	int16_t flow_comp_m_y;// y velocity*1000 in meters / timestep
	int16_t qual;// Optical flow quality / confidence 0: bad, 255: maximum quality
	int16_t gyro_x_rate; //gyro x rate
	int16_t gyro_y_rate; //gyro y rate
	int16_t gyro_z_rate; //gyro z rate
	uint8_t gyro_range; // gyro range
	uint8_t sonar_timestamp;// timestep in milliseconds between I2C frames
	int16_t ground_distance;// Ground distance in meters*1000. Positive value: distance known. Negative value: Unknown distance
} PX4_data;

typedef union _PX4_frame
{
	char raw_8[sizeof(PX4_data)];
	PX4_data data;
} PX4_frame;

enum PX4_State_t
{
	UNINITIALIZED,
	SENDING_REQ_BYTE,
	WAITING_FOR_RESPONSE,
	GET_FIRST_BYTE,
	GET_INTERMEDIATE_BYTES,
	GET_LAST_BYTE,
	DATA_READY
};

extern volatile enum PX4_State_t g_PX4_State;
extern volatile uint32_t idx_px4;
extern volatile uint32_t I2C_Base;
extern volatile bool dataFresh;

//volatile enum PX4_State_t g_PX4_State;
//volatile uint32_t idx_px4;
//volatile uint32_t I2C_Base;
//volatile bool dataFresh;

void initiate_PX4_transmit(void);
void init_px4_i2c(uint32_t I2C_peripheral , uint32_t I2C_SCL_peripheral ,
				  uint32_t I2C_SDA_peripheral, uint32_t sysClock,
				  uint32_t _I2C_base, uint32_t SCL_portBase,
				  uint32_t SDA_portBase, uint32_t SCL_pin, uint32_t SDA_pin,
				  uint32_t SCL_pinConfig, uint32_t SDA_pinConfig);
bool px4_i2c_dataFresh(void);
//int16_t px4_i2c_getFlowX(void);
//int16_t px4_i2c_getFlowY(void);
int16_t px4_i2c_getHeight(void);
uint8_t px4_i2c_getTimestep(void);
void px4_i2c_makeDataStale(void);


uint16_t px4_i2c_get_frame_count();
int16_t  px4_i2c_get_pixel_flow_x_sum();
int16_t  px4_i2c_get_pixel_flow_y_sum();
int16_t  px4_i2c_get_flow_comp_m_x();
int16_t  px4_i2c_get_flow_comp_m_y();
int16_t  px4_i2c_get_qual();
int16_t  px4_i2c_get_gyro_x_rate();
int16_t  px4_i2c_get_gyro_y_rate();
int16_t  px4_i2c_get_gyro_z_rate();
uint8_t  px4_i2c_get_gyro_range();



#endif /* PX4_I2C_H_ */

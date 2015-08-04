/*
 * I2C_defines.hpp
 *
 *  Created on: Jun 24, 2015
 *      Author: Zhengjie
 */

#ifndef I2CDEFINES_HPP_
#define I2CDEFINES_HPP_

/***************************Command, Register and Address**********************/
//#define PX4_I2C_ADDRESS			0x43		// Define PX4 Address
#define PX4_I2C_ADDRESS			0x46
#define PX4_MEA     			0x00		// Measurement Command
#define LIDAR_I2C_ADDRESS      	0x62		// Define Lidar I2C Address.
#define LIDAR_REG_MEA     		0x00        // Register to write to initiate ranging.
#define LIDAR_MEA_VALE	        0x04        // Value to initiate ranging.
#define LIDAR_FULL_BYTE		    0x8F        // Register to get both High and Low bytes in 1 call.

#endif /* I2CDefines.hpp */

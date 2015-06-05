/*
 * imu_defines.hpp
 *
 *  Created on: May 28, 2015
 *      Author: zjcui
 */

#ifndef IMU_DEFINES_HPP_
#define IMU_DEFINES_HPP_

//#include "inc/hw_ints.h"
//#include "inc/hw_memmap.h"
//#include "inc/hw_types.h"
//#include "driverlib/gpio.h"
//#include "driverlib/pin_map.h"


#define NUM_M_VAL 9
#define IMU_DATA_LENGTH 79

#define IMU_UART_BASE UART1_BASE
#define PC_UART_BASE UART0_BASE
//#define IMU_GPIO_PORT SYSCTL_PERIPH_GPIOC
//#define IMU_UART_RX GPIO_PC4_U1RX
//#define IMU_UART_TX GPIO_PC5_U1TX

#define RED_LED GPIO_PIN_1
#define BLUE_LED GPIO_PIN_2
#define GREEN_LED GPIO_PIN_3

#endif /* IMU_DEFINES_HPP_ */

#ifndef IMU_HW_HPP_
#define IMU_HW_HPP_

#include "Imu_Defines.hpp"

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"

#include "utils/uartstdio.h"

#define ALL_VAL_CMD 0xCC
#define EULER_ANGRATES_CMD 0xCF

// Global data to send and receive for testing
extern uint8_t imuRawFinal[IMU_DATA_LENGTH];
extern uint8_t imuRawIn[IMU_DATA_LENGTH];
extern bool imuDone;
extern uint32_t imu_time;
extern uint8_t imuIndex;
extern uint8_t imuCmd;

// Configure Sys Clock.
void imu_uart_config_sys_clock(void);

// Configure UART.
void imu_uart_config_uart(void);

void imu_uart_int_handler(void);

void imu_uart_send(uint32_t, const uint8_t *, uint32_t);

// For debug
void imu_uart_config_LED(void);

void imu_uart_toggle_LED(uint32_t, uint32_t);

void imu_uart_timer_init(void);

void imu_memcpy(uint8_t*, const uint8_t*, int);

#endif /* IMU_HW_HPP */

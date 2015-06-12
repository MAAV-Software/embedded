#include "imu_defines.hpp"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"

#define ALL_VAL_CMD 0xCC
#define EULER_ANGRATES_CMD 0xCF

// Global data to send and receive for testing
extern uint8_t imuRawFinal[IMU_DATA_LENGTH];
extern uint8_t imuRawWorking[IMU_DATA_LENGTH];
extern bool imuReady;


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

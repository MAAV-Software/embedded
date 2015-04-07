#ifndef GENERAL_H_
#define GENERAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c123gh6pm.h"

#include "driverlib/gpio.h"
#include "driverlib/fpu.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/eeprom.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"

// System Clock Frequency
#define SYSCLOCK 80000000

// PX4 Stuff
#define PX4_PERIPH SYSCTL_PERIPH_I2C3
#define PX4_SCL_PERIPH SYSCTL_PERIPH_GPIOD
#define PX4_SDA_PERIPH SYSCTL_PERIPH_GPIOD
#define PX4_I2C_BASE I2C3_BASE
#define PX4_SCL_BASE GPIO_PORTD_BASE
#define PX4_SDA_BASE GPIO_PORTD_BASE
#define PX4_SCL_PIN GPIO_PIN_0
#define PX4_SDA_PIN GPIO_PIN_1
#define PX4_SCL_PIN_CONFIG GPIO_PD0_I2C3SCL
#define PX4_SDA_PIN_CONFIG GPIO_PD1_I2C3SDA

#ifdef __cplusplus
}
#endif


#endif /* GENERAL_H_ */

/*
 * I2C_hw.hpp
 *
 *  Created on: Jun 24, 2015
 *      Author: Zhengjie
 */

#ifndef I2CHW_HPP_
#define I2CHW_HPP_

#include <stdint.h>
#include "sensorlib/i2cm_drv.h"
#include "I2CDefines.hpp"
#include "Px4Defines.hpp"
#include "LidarDefines.hpp"

// The I2C master driver instance data.
extern tI2CMInstance I2CMInst;

// A boolean that is set when an I2C transaction is completed.
extern bool I2CMDone;

extern uint32_t i2cBase;

//***************************State Machine**************************************
// Keep track of state machine, meaning the state just done
typedef enum _state { Lidar_1, Lidar_2, PX4_1 } State;
volatile State nextState, currentState;
extern uint32_t LidarTime, PX4Time;

extern uint8_t command[4];

extern uint8_t rawPx4[sizeof(px4Frame)];
extern uint8_t rawLidar[LIDAR_DIST_SIZE];


// The function that is provided by this Run as a callback when I2C transactions have completed.
void I2CMCallback(void *pvData, uint_fast8_t ui8Status);

void ConfigI2C(const uint32_t sysctlPeriphI2C,
			   const uint32_t sysctlPeriphGPIO,
			   const uint32_t sclPinConfig,
			   const uint32_t sdaPinConfig,
			   const uint32_t gpioBase,
			   const uint32_t gpioSclPin,
			   const uint32_t gpioSdaPin,
			   const uint32_t _i2cBase,
			   const uint32_t i2cInterrupt,
			   const bool useFastBus);

#endif /* I2CHW.hpp */

#ifndef IMUHW_HPP_
#define IMUHW_HPP_

#include "ImuDefines.hpp"
#include <stdint.h>

// Global data to send and receive for testing
extern uint8_t imuRawFinal[IMU_DATA_LENGTH];
extern uint8_t imuRawIn[IMU_DATA_LENGTH];
extern bool imuDone;
extern uint32_t imuTime;
extern uint8_t imuIndex;
extern uint8_t imuCmd;

// Configure UART.
void imuUartConfig(const uint32_t sysctlPeriphUart,
				   const uint32_t sysctlPeriphGPIO,
				   const uint32_t rxPinConfig,
				   const uint32_t txPinConfig,
				   const uint32_t gpioPortBase,
				   const uint32_t gpioRxPin,
				   const uint32_t gpioTxPin,
				   const uint32_t _uartBase,
				   const uint32_t uartInterrupt);

void imuUartIntHandler();

void imuUartSend(const uint8_t *buffer, uint32_t count);

void imuMemcpy(uint8_t* out, const uint8_t* in, const int len);

void imuCalibrate(uint32_t AccelBiasX, uint32_t AccelBiasY, uint32_t AccelBiasZ);

#endif /* IMUHW_HPP */

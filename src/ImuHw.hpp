#ifndef IMUHW_HPP_
#define IMUHW_HPP_

#include "ImuDefines.hpp"
#include <stdint.h>

// Global data to send and receive for testing
extern uint8_t IMU_RAW_DATA[MAX_IMU_DATA_LENGTH];
extern bool IMU_DONE;

// Configure UART
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

/*
 * PRECONDITION: Don't call while in the middle of reading in another message.
 */
void imuUartSend(const uint8_t *buffer, uint32_t count);

//void imuMemcpy(uint8_t* out, const uint8_t* in, const int len);

#endif /* IMUHW_HPP */

#ifndef DATALINKHW_HPP_
#define DATALINKHW_HPP_

#include <stdint.h>
#include "messaging/RingBuffer.hpp"

extern RingBuffer<256> DL_RBUFF;

void DataLinkUartConfig(const uint32_t sysctlPeriphUart,
						const uint32_t sysctlPeriphGPIO,
						const uint32_t rxPinConfig,
						const uint32_t txPinConfig,
						const uint32_t gpioPortBase,
						const uint32_t gpioRxPin,
						const uint32_t gpioTxPin,
						const uint32_t _uartBase,
						const uint32_t uartInterrupt);

void DataLinkUartSend(const uint8_t *buf, uint32_t len);

void DataLinkUartIntHandler();

#endif /* DataLinkHw.hpp */

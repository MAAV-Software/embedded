#ifndef DATALINKHW_HPP_
#define DATALINKHW_HPP_

#include <stdint.h>

#define DLINK_UART_BUF_LEN 8

extern bool DLINK_RX_DONE;
extern uint8_t DLINK_RX_BUF[DLINK_UART_BUF_LEN];


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


#endif /* DataLinkHw.hpp */

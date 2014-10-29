/*
 * px4_i2c.c
 *
 *  Created on: Jul 7, 2014
 *      Author: Jonathan Kurzer
 */
#include <stdbool.h>
#include <stdint.h>

#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"

#include "utils/uartstdio.h"

#include "px4_i2c.h"

volatile PX4_frame px4;
volatile PX4_frame px4_buffer;

volatile enum PX4_State_t g_PX4_State;
volatile uint32_t idx_px4;
volatile uint32_t I2C_Base;
volatile bool dataFresh;

static void master_I2C3_Int_Handler(void) {
	I2CMasterIntClear(I2C_Base);
	uint32_t errVal = I2CMasterErr(I2C_Base);
//	if (errVal != I2C_MASTER_ERR_NONE) {
		// I2C error
//		GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1, 0);
//		SysCtlDelay(1000);
//		return;
//	}
	char recvChar = I2CMasterDataGet(I2C_Base);

	if(g_PX4_State == GET_FIRST_BYTE) {	// First byte just came in.
		px4.raw_8[idx_px4++] = recvChar;			// Put first byte in the array
		g_PX4_State = GET_INTERMEDIATE_BYTES;
		I2CMasterControl(I2C_Base, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
	} else if(g_PX4_State == GET_INTERMEDIATE_BYTES) {
		px4.raw_8[idx_px4++] = recvChar;	// Put an intermediate byte in the array
		if(idx_px4 == 21) {					// this was the second to last byte
			g_PX4_State = GET_LAST_BYTE;
			I2CMasterControl(I2C_Base, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
		} else {
			I2CMasterControl(I2C_Base, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
		}
	} else if(g_PX4_State == GET_LAST_BYTE) {
		px4.raw_8[idx_px4] = recvChar;		// Put last byte in the array
		g_PX4_State = DATA_READY;
		if(dataFresh == false)
		{
			memcpy((void*)&px4_buffer, (void*)&px4, sizeof(PX4_frame));
			dataFresh = true;
		}
	} else if(g_PX4_State == DATA_READY) {
		// There is no reason I should get here
		g_PX4_State = DATA_READY;
	} else if(g_PX4_State == SENDING_REQ_BYTE) {
		idx_px4 = 0;
		g_PX4_State = GET_FIRST_BYTE;
		I2CMasterSlaveAddrSet(I2C_Base, PX4_ADDRESS, true);
		I2CMasterControl(I2C_Base, I2C_MASTER_CMD_BURST_RECEIVE_START);
	}
	return;
}

void init_px4_i2c(
		uint32_t I2C_peripheral, uint32_t I2C_SCL_peripheral, uint32_t I2C_SDA_peripheral,
		uint32_t sysClock, uint32_t _I2C_base, uint32_t SCL_port, uint32_t SDA_port,
		uint32_t SCL_pin, uint32_t SDA_pin, uint32_t SCL_pinConfig, uint32_t SDA_pinConfig) {

	I2C_Base = _I2C_base;
	SysCtlPeripheralEnable(I2C_peripheral);
	SysCtlPeripheralEnable(I2C_SCL_peripheral);
	SysCtlPeripheralEnable(I2C_SDA_peripheral);

	GPIOPinConfigure(SCL_pinConfig);
	GPIOPinConfigure(SDA_pinConfig);

	GPIOPinTypeI2CSCL(SCL_port, SCL_pin);
	GPIOPinTypeI2C(SDA_port, SDA_pin);

	I2CMasterInitExpClk(I2C_Base, sysClock, true);  // set SCK to 400 Kbps

	I2CIntRegister(I2C_Base, master_I2C3_Int_Handler);

	I2CMasterIntEnableEx(I2C_Base, I2C_MASTER_INT_DATA);

	I2CMasterSlaveAddrSet(I2C_Base, PX4_ADDRESS, false);

	g_PX4_State = UNINITIALIZED;
	dataFresh = false;
}
void initiate_PX4_transmit(void) {
	g_PX4_State = SENDING_REQ_BYTE;
	I2CMasterDataPut(I2C_Base, 0);
	I2CMasterControl(I2C_Base, I2C_MASTER_CMD_SINGLE_SEND);
	return;
}
bool px4_i2c_dataFresh(void) {
	return dataFresh ? true : false;
}
//int16_t px4_i2c_getFlowX(void) {
//	return px4.data.flow_comp_m_x;
//}
//int16_t px4_i2c_getFlowY(void) {
//	return px4.data.flow_comp_m_y;
//}
uint16_t px4_i2c_get_frame_count() {
	return px4_buffer.data.frame_count;
}
int16_t px4_i2c_get_pixel_flow_x_sum() {
	return px4_buffer.data.pixel_flow_x_sum;
}
int16_t px4_i2c_get_pixel_flow_y_sum() {
	return px4_buffer.data.pixel_flow_y_sum;
}
int16_t px4_i2c_get_flow_comp_m_x() {
	return px4_buffer.data.flow_comp_m_x;
}
int16_t px4_i2c_get_flow_comp_m_y() {
	return px4_buffer.data.flow_comp_m_y;
}
int16_t px4_i2c_get_qual() {
	return px4_buffer.data.qual;
}
int16_t px4_i2c_get_gyro_x_rate() {
	return px4_buffer.data.gyro_x_rate;
}
int16_t px4_i2c_get_gyro_y_rate() {
	return px4_buffer.data.gyro_y_rate;
}
int16_t px4_i2c_get_gyro_z_rate() {
	return px4_buffer.data.gyro_z_rate;
}
uint8_t px4_i2c_get_gyro_range() {
	return px4_buffer.data.gyro_range;
}
uint8_t px4_i2c_getTimestep(void) {
	return px4_buffer.data.sonar_timestamp;
}
int16_t px4_i2c_getHeight(void) {
	return px4_buffer.data.ground_distance;
}

void px4_i2c_makeDataStale(void) {	// TODO
	dataFresh = false;
	return;
}

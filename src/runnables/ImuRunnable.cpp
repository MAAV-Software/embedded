/*
 * ImuRunnable.cpp
 *
 *  Created on: Jul 10, 2015
 *      Author: Zhengjie
 */

#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "runnables/ImuRunnable.hpp"
#include "ImuHw.hpp"

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

#include "utils/uartstdio.h"

#include "time_util.h"


float ReverseBytes(const uint8_t *raw, const unsigned int i)
{
	union B2F
	{
	   unsigned char buf[4];
	   float number;
	} data;

	data.buf[0] = raw[i+3];
	data.buf[1] = raw[i+2];
	data.buf[2] = raw[i+1];
	data.buf[3] = raw[i+0];

	return data.number;

}

void ImuRunnable::AccelCalib()
{

	    imuCmd = MaavImu::ImuMeasurementCommand;
		uint32_t getTime = MAP_TimerValueGet(TIMER4_BASE, TIMER_A);
		float sysClock = (float)SYSCLOCK;

		//Accumulate the bias over 10 measurements
		float accumulatedAccX = 0;
		float accumulatedAccY = 0;
		float accumulatedAccZ = 0;

		for(int i = 0; i < 10; i++)
		{
			imuUartSend(&imuCmd,1)
		    while(!imuDone && ((getTime - imuTime) > (sysClock / 1000.0 * 10.0)))
		    {
		    	imuTime = getTime;
		    }
			state->imu->parse(imuRawFinal);
			accumulatedAccX = accumulatedAccX + state->imu->getAccX();
			accumulatedAccY = accumulatedAccY + state->imu->getAccY();
			accumulatedAccZ = accumulatedAccZ + state->imu->getAccZ();
		}

		accumulatedAccX = accumulatedAccX/10;
		accumulatedAccY = accumulatedAccY/10;
		accumulatedAccZ = accumulatedAccZ/10 -1;


		struct P{
				uint8_t B1;
				uint8_t B2;
				uint8_t B3;
				float AccelBiasX;
				float AccelBiasY;
				float AccelBiasZ;
		};

		union Packet
		{
			    P p_packet;
				uint8_t raw[sizeof(P)];
		}packet;

		packet.p_packet.B1 = 0xC9;
		packet.p_packet.B2 = 0xB7;
		packet.p_packet.B3 = 0x44;
		packet.p_packet.AccelBiasX = accumulatedAccX;
		packet.p_packet.AccelBiasY = accumulatedAccY;
		packet.p_packet.AccelBiasZ = accumulatedAccZ;

		//TODO: Fix Reversing of Bytes
		//Reverse the sequence of bytes -NEED To Fix
		packet.p_packet.AccelBiasX = ReverseBytes(packet.raw,3);
		packet.p_packet.AccelBiasY = ReverseBytes(packet.raw,7);
		packet.p_packet.AccelBiasZ = ReverseBytes(packet.raw,10);

		imuUartSend(packet.raw,sizeof(packet.p_packet));

		while(!imuDone && ((getTime - imuTime) > (sysClock / 1000.0 * 10.0)))
		{
		   	imuTime = getTime;
		}

		state->imu->parse(imuRawFinal);

		if(accumulatedAccX != state->imu->getAccBiasX() || accumulatedAccY != state->imu->getAccBiasY() || accumulatedAccY == state->imu->getAccBiasY()){
			//response bad
			goApeshit();
		}
}

void ImuRunnable::CaptureGyroBias(){

    imuCmd = MaavImu::GyroCalibCommand;
	uint32_t getTime = MAP_TimerValueGet(TIMER4_BASE, TIMER_A);
	float sysClock = (float)SYSCLOCK;


	struct P{
			uint8_t B1;
			uint8_t B2;
			uint8_t B3;
			uint16_t SamplingTime;

	};

	union Packet
	{
		    P p_packet;
			uint8_t raw[sizeof(P)];
	}packet;

	packet.p_packet.B1 = 0xCD;
	packet.p_packet.B2 = 0xC1;
	packet.p_packet.B3 = 0x29;
	packet.p_packet.SamplingTime = 20; //TODO: Confirm this value


	imuUartSend(packet.raw,sizeof(packet.p_packet));

	while(!imuDone && ((getTime - imuTime) > (sysClock / 1000.0 * 10.0)))
	{
	   	imuTime = getTime;
	}

	state->imu->parse(imuRawFinal);
}



ImuRunnable::ImuRunnable(ProgramState *pState) : state(pState)
{
	// may want to move to main
	AccelCalib();
	CaptureGyroBias();

	imuUartConfig(SYSCTL_PERIPH_UART1, SYSCTL_PERIPH_GPIOC, GPIO_PC4_U1RX,
				  GPIO_PC5_U1TX, GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_5,
				  UART1_BASE, INT_UART1);
}

void ImuRunnable::run()
{
	uint32_t getTime = MAP_TimerValueGet(TIMER4_BASE, TIMER_A);
	float sysClock = (float)SYSCLOCK;
	float time;	

	// Run at 100Hz 10ms
	if (!imuDone && ((getTime - imuTime) > (sysClock / 1000.0 * 10.0)))
	{
		imuTime = getTime;
		imuUartSend(&imuCmd, 1);
	}

	if (imuDone)
	{
		time = (float)millis() / 1000.0f;
		state->imu->RecordTime(time);
		state->imu->parse(imuRawFinal);
		imuDone = false;
	}
}

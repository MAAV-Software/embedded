/*
 * ImuRunnable.cpp
 *
 *  Created on: Jul 10, 2015
 *      Author: Zhengjie
 */

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <cmath>

#include "runnables/ImuRunnable.hpp"
#include "ImuHw.hpp"
#include "Imu.hpp"
#include "ImuDefines.hpp"

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
#include "LED.h"
#include "Apeshit.hpp"
#include "MaavMath"

using MaavMath::Gravity;
using namespace MaavImu;

void ImuRunnable::AccelCalib()
{
    //Accumulate the bias over 10 measurements
    float accXBias = 0;
    float accYBias = 0;
    float accZBias = 0;
    
    TurnOff_LED(BLUE_LED);

    // sample IMU at 10 ms period for 10 samples    
    uint8_t numSamples = 0;
    uint8_t numLoops = 11;
    for (uint8_t i = 0; i < numLoops; ++i)
    {
        MicroStrainCmd measCmd = state->imu->formatMeasCmd();
        imuUartSend(measCmd.buf, measCmd.length);
        
        while (!IMU_DONE) TurnOff_LED(RED_LED); // busy wait OK here
        TurnOn_LED(RED_LED);

        if (i == 0) continue;

        ++numSamples;
        // parse the measrement message
        state->imu->parse(IMU_RAW_DATA);
        IMU_DONE = false;

        accXBias += state->imu->getgAccX();
        accYBias += state->imu->getgAccY();
        accZBias += state->imu->getgAccZ() + 1;
        
        MAP_SysCtlDelay(SYSCLOCK / 3 / 100 * 2);
    }
    TurnOn_LED(BLUE_LED);

    TurnOff_LED(GREEN_LED);
    
    if (numSamples > 0)
    {
    	// normalize by numSamples
    	accXBias /= (float)numSamples;
    	accYBias /= (float)numSamples;
    	accZBias /= (float)numSamples;
    }

    IMU_DONE = false;

    // format and send command to register accel bias in the IMU
    MicroStrainCmd accBiasCmd = state->imu->formatAccelBiasCmd(accXBias, 
                                    accYBias, accZBias);
    imuUartSend(accBiasCmd.buf, accBiasCmd.length);

    while (!IMU_DONE) TurnOff_LED(RED_LED); // busy loop here is OK
    TurnOn_LED(RED_LED);
    
    // parse the bias message
    state->imu->parse(IMU_RAW_DATA);
    IMU_DONE = false;

    // verify that biases were registered
    if ((accXBias != state->imu->getAccBiasX()) ||
        (accYBias != state->imu->getAccBiasY()) ||
        (accZBias != state->imu->getAccBiasZ()))
    {
        goApeshit();
    }

    TurnOn_LED(GREEN_LED);
}

void ImuRunnable::CaptureGyroBias()
{
    // command MicroStrain to capture gyro biases with 10 s (10000 us) sampling
    // time
    MicroStrainCmd cmd = state->imu->formatGyroBiasCmd(10000);
    imuUartSend(cmd.buf, cmd.length);

    while (!IMU_DONE) TurnOff_LED(RED_LED); // busy loop here is OK
    TurnOn_LED(RED_LED);
    
    // parse the bias message
    state->imu->parse(IMU_RAW_DATA);
    IMU_DONE = false;
}

ImuRunnable::ImuRunnable(ProgramState *pState) : state(pState), imuTime(0)
{
	// THESE ARE FOR BOTH 2015 AND 2016 SIGNAL BOARDS
	imuUartConfig(SYSCTL_PERIPH_UART1, SYSCTL_PERIPH_GPIOC, GPIO_PC4_U1RX,
				  GPIO_PC5_U1TX, GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_5,
				  UART1_BASE, INT_UART1);
	
	softReset();

	setMode();

    AccelCalib();

	Toggle_LED(BLUE_LED | GREEN_LED, SYSCLOCK / 3 / 2);

	CaptureGyroBias();

	for (int i = 0; i < 3; ++i) Toggle_LED(BLUE_LED, SYSCLOCK / 3 / 2);
}

void ImuRunnable::run()
{
	uint32_t time = MAP_TimerValueGet(TIMER4_BASE, TIMER_A);

	// Run at 100Hz (10ms)
	if (!IMU_DONE && ((time - imuTime) > (SYSCLOCK / 1000 * 10)))
	{
		imuTime = time;
        MicroStrainCmd cmd = state->imu->formatMeasCmd();
		imuUartSend(cmd.buf, cmd.length);
	}

	if (IMU_DONE) // parse the measurement message if done
	{
		state->imu->RecordTime((float)millis() / 1000.0f);
		state->imu->parse(IMU_RAW_DATA);
		IMU_DONE = false;

		state->dLink->send(state->imu->getImuData());
		/*
		imu_t imuMsg;
		imuMsg.AccX = state->imu->getAccX();
		imuMsg.AccY = state->imu->getAccY();
		imuMsg.AccZ = state->imu->getAccZ();
		imuMsg.AngRateX = state->imu->getAngRateX();
		imuMsg.AngRateY = state->imu->getAngRateY();
		imuMsg.AngRateZ = state->imu->getAngRateZ();
		imuMsg.roll = state->imu->getRoll();
		imuMsg.pitch = state->imu->getPitch();
		imuMsg.yaw = state->imu->getYaw();
		imuMsg.timestamp = state->imu->getTimestamp();
		state->dLink->send(&imuMsg);
		*/
	}
}

void ImuRunnable::setMode()
{
	MicroStrainCmd cmd = state->imu->formatStopContMode();
	imuUartSend(cmd.buf, cmd.length);
	Toggle_LED(BLUE_LED | GREEN_LED, SYSCLOCK / 3 / 2);

	IMU_DONE = false;
}

void ImuRunnable::softReset()
{
	MicroStrainCmd cmd = state->imu->formatSoftResetCmd();
	imuUartSend(cmd.buf, cmd.length);
	Toggle_LED(BLUE_LED | GREEN_LED, SYSCLOCK / 3 / 2);

	IMU_DONE = false;
}

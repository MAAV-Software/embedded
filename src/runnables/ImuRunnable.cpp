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

using namespace MaavImu;

void ImuRunnable::AccelCalib()
{
    //Accumulate the bias over 10 measurements
    float accXBias = 0;
    float accYBias = 0;
    float accZBias = 0;

    uint32_t startTime = MAP_TimerValueGet(TIMER4_BASE, TIMER_A);
    uint32_t period = SYSCLOCK / 100; // period of 10 ms
    
    // sample IMU at 10 ms period for 10 samples    
    uint8_t numSamples = 10;
    for (uint8_t i = 0; i < numSamples; ++i)
    {
        uint32_t startTime = millis();

        MicroStrainCmd measCmd = state->imu->formatMeasCmd();
        imuUartSend(measCmd.buf, measCmd.length)
        
        while (!IMU_DONE); // busy wait OK here
        
        // parse the measrement message
        state->imu->parse(IMU_RAW_DATA);
        IMU_DONE = false;
        
        accXBias += state->imu->getAccX();
        accYBias += state->imu->getAccY();
        accZBias += state->imu->getAccZ();
        
        uint32_t currTime = millis();
        
        // delay the remainder of the 10 ms period
        MAP_SysCtlDelay(SYSCLOCK / 300 * (period - currTime - startTime));
    }
    
    // normalize by numSamples
    accXBias /= (float)numSamples;
    accYBias /= (float)numSamples;
    accZBias /= (float)numSamples;
    accZBias -= 1.0; // subtract gravity
    
    // format and send command to register accel bias in the IMU
    MicroStrainCmd accBiasCmd = state->imu->formatAccelBiasCmd(accXBias, 
                                    accYBias, accZBias);
    imuUartSend(accBiasCmd.buf, accBiasCmd.length);

    while (!IMU_DONE); // busy loop here is OK
    
    // parse the bias message
    state->imu->parse(IMU_RAW_DATA);
    IMU_DONE = false;

    // verify that biases were registered
    if ((accXBias != state->imu->getAccBiasX) || 
        (accYBias != state->imu->getAccBiasY) || 
        (accZBias != state->imu->getAccBiasZ))
    {
        goApeshit();
    }
}

void ImuRunnable::CaptureGyroBias()
{
    // command MicroStrain to capture gyro biases with 10 s (10000 us) sampling
    // time
    MicroStrainCmd cmd = state->imu->formatGyroBiasCmd(10000);
    imuUartSend(cmd.buf, cmd.length);

    while (!IMU_DONE); // busy loop here is OK
    
    // parse the bias message
    state->imu->parse(IMU_RAW_DATA);
    IMU_DONE = false;
}

ImuRunnable::ImuRunnable(ProgramState *pState) : state(pState), imuTime(0)
{
	imuUartConfig(SYSCTL_PERIPH_UART1, SYSCTL_PERIPH_GPIOC, GPIO_PC4_U1RX,
				  GPIO_PC5_U1TX, GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_5,
				  UART1_BASE, INT_UART1);
	
	CaptureGyroBias();
    AccelCalib();
	
    for (int i = 0; i < 2; ++i) Toggle_LED(BLUE_LED, SYSCLOCK / 3 / 2);
}

void ImuRunnable::run()
{
	uint32_t time = MAP_TimerValueGet(TIMER4_BASE, TIMER_A);

	// Run at 100Hz (10ms)
	if (!IMU_DONE && ((time - imuTime) > (SYSCLOCK / 100)))
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
	}
}

/*
 * main.c
 */
#include "arm_math.h"
#include <stdlib.h>
#include "ExtendedKalmanFilter.hpp"
#include "KalmanFunctions.hpp"

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c123gh6pm.h"

#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"

int main(void) {
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
			SYSCTL_XTAL_16MHZ);
	float initialState[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
	float initialErrorCov[81]; memset(initialErrorCov, 0, sizeof(float) * 81);
	ExtendedKalmanFilter filter(9, initialState, initialErrorCov);

	float predictCov[81] = {
			1, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 1, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 1, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 1, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 1, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 1, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 1, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 1, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 1,
	};
	arm_matrix_instance_f32 predictCovMat;
	arm_mat_init_f32(&predictCovMat, 9, 9, predictCov);
	filter.setPredictFunc(4, systemDeltaState, systemGetJacobian, &predictCovMat);

	float updateCov[36] = {
			1, 0, 0, 0, 0, 0,
			0, 1, 0, 0, 0, 0,
			0, 0, 1, 0, 0, 0,
			0, 0, 0, 1, 0, 0,
			0, 0, 0, 0, 1, 0,
			0, 0, 0, 0, 0, 1,
	};
	arm_matrix_instance_f32 updateCovMat;
	arm_mat_init_f32(&updateCovMat, 6, 6, updateCov);
	filter.setUpdateFunc(0, 6, sensorPredict, sensorGetJacobian, &updateCovMat);

	float controlInput[4] = {0, 0, 0, 0};
	arm_matrix_instance_f32 controlInputMat;
	arm_mat_init_f32(&controlInputMat, 4, 1, controlInput);

	float sensorMeasurement[6] = {0, 0, 0, 0, 0, 0};
	arm_matrix_instance_f32 sensorMeasurementMat;
	arm_mat_init_f32(&sensorMeasurementMat, 6, 1, sensorMeasurement);

	while(1) {
		filter.predict(0.1, &controlInputMat);
		filter.update(0.1, 0, &sensorMeasurementMat);
	}
	return 0;
}

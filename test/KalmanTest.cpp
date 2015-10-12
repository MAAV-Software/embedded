#include <iostream>
#include <stdint.h>
#include "cmeigen.hpp"
#include "kalman/ExtendedKalmanFilter.hpp"
#include "kalman/KalmanFunctions.hpp"

using namespace std;

int main()
{
	uint16_t numStates = 6;
	uint16_t numCtrlInputs = 4;
	float initState[6] = {0, 0, 0, 0, 0, 0};
	float initP[36] = {
		0.1f, 0, 0, 0, 0, 0,
		0, 0.1f, 0, 0, 0, 0,
		0, 0, 0.1f, 0, 0, 0,
		0, 0, 0, 0.1f, 0, 0,
		0, 0, 0, 0, 0.1f, 0,
		0, 0, 0, 0, 0, 0.1f
	};
	float Qdata[36] = {
		0.1f, 0, 0, 0, 0, 0,
		0, 0.1f, 0, 0, 0, 0,
		0, 0, 0.1f, 0, 0, 0,
		0, 0, 0, 0.09f, 0, 0,
		0, 0, 0, 0, 0.09f, 0,
		0, 0, 0, 0, 0, 0.1f
	};
	float RdataNoCam[9] = {
		 0.01f,   0,   0,
		   0, 0.05f,   0,
	  	   0,   0, 0.05f
	};
	
	// create filter with inital state and P buffers
	ExtendedKalmanFilter ekf(numStates, initState, initP);

	// create Q and set the prediction using Kalman Function ptrs
	arm_matrix_instance_f32 Q;
	arm_mat_init_f32(&Q, numStates, numStates, Qdata);
	ekf.setPredictFunc(numCtrlInputs, systemDeltaState, systemGetJacobian, &Q);
	
	// create R and set the update using Kalman Function ptrs
	uint16_t numSensorsNoCam = 3;
	uint16_t noCamId = 0;
	arm_matrix_instance_f32 RNoCam;
	arm_mat_init_f32(&RNoCam, numSensorsNoCam, numSensorsNoCam, RdataNoCam);
	ekf.setUpdateFunc(noCamId, numSensorsNoCam, sensorPredict, sensorGetJacobian, &RNoCam);
	
	// set up u column vector of control inputs
	float ctrlInData[4] = {0, 0, 0, 0};
	arm_matrix_instance_f32 ctrlIn;
	arm_mat_init_f32(&ctrlIn, numCtrlInputs, 1, ctrlInData);

	// set up y column vector of sensor measurements
	float sensorsNoCamData[3] = {0, 0, 0};
	arm_matrix_instance_f32 sensorsNoCam;
	arm_mat_init_f32(&sensorsNoCam, numSensorsNoCam, 1, sensorsNoCamData);
	
	float dt = 0.010f;
	float mass = 2.50f;
	float gravity = 9.81f;
	arm_matrix_instance_f32 x;
	
	// run prediction
	ctrlIn.pData[0] = (mass * gravity) + 0.01f;
	ctrlIn.pData[1] = 0.05f;
	ekf.predict(dt, mass, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f, &ctrlIn);
	x = ekf.getState();

	cout << "X after Prediction with dt = " << dt << " and u1 = " << ctrlIn.pData[0] << endl;
	for (uint16_t i = 0; i < x.numRows; ++i) cout << x.pData[i] << " ";
	cout << endl;

	// run correction
	sensorsNoCam.pData[0] = 1.0f;
	sensorsNoCam.pData[1] = 1.5f;
	ekf.update(dt, noCamId, &sensorsNoCam);
	x = ekf.getState();
	
	cout << "Sensor input [z, xdot, ydot] = ";
	for (uint16_t i = 0; i < sensorsNoCam.numRows; ++i) cout << sensorsNoCam.pData[i] << " ";
	cout << endl;
	
	cout << "X after Correction = " << endl;
	for (uint16_t i = 0; i < x.numRows; ++i) cout << x.pData[i] << " ";
	cout << endl;

	// run prediction
	ctrlIn.pData[0] = (mass * gravity) + 0.01f;
	ctrlIn.pData[1] = 0.0f;
	ekf.predict(dt, mass, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f, &ctrlIn);
	x = ekf.getState();

	cout << "X after Prediction 2 with dt = " << dt << " and u1 = " << ctrlIn.pData[0] << endl;
	for (uint16_t i = 0; i < x.numRows; ++i) cout << x.pData[i] << " ";
	cout << endl;
	
	ctrlIn.pData[0] = (mass * gravity) + 0.01f;
	ctrlIn.pData[1] = 2.0f;
	ekf.predict(dt, mass, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f, &ctrlIn);
	x = ekf.getState();

	cout << "X after Prediction 3 with dt = " << dt << " and u1 = " << ctrlIn.pData[0] << endl;
	for (uint16_t i = 0; i < x.numRows; ++i) cout << x.pData[i] << " ";
	cout << endl;
	
	// run correction
	sensorsNoCam.pData[0] = 1.5f;
	sensorsNoCam.pData[1] = 1.0f;
	ekf.update(dt, noCamId, &sensorsNoCam);
	x = ekf.getState();
	
	cout << "Sensor input 2 [z, xdot, ydot] = ";
	for (uint16_t i = 0; i < sensorsNoCam.numRows; ++i) cout << sensorsNoCam.pData[i] << " ";
	cout << endl;
	
	cout << "X after Correction 2 = " << endl;
	for (uint16_t i = 0; i < x.numRows; ++i) cout << x.pData[i] << " ";
	cout << endl;
	
	return 0;
}

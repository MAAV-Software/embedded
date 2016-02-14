#include <iostream>
#include <fstream>
#include <stdint.h>
#include <math.h>
#include "cmeigen.hpp"
#include "kalman/ExtendedKalmanFilter.hpp"
#include "kalman/KalmanFunctions.hpp"

using namespace std;

int main(int argc, char **argv)
{
	if(argc < 2) return 1;
	ifstream logfile(argv[1]);//open first parameter as log file
	ofstream outfile(argv[2]);//open second parameter as output file
	if(!logfile) return 1; //could not open

	//logic of whether to predict or correct
	float lastTime = 0;
	bool usePredict = false;

	float Time          ;
	float Mode          ;
	float Imu_AccX      ;
	float Imu_AccY      ;
	float Imu_AccZ      ;
	float Imu_AngRateX  ;
	float Imu_AngRateY  ;
	float Imu_AngRateZ  ;
	float Imu_MagX      ;
	float Imu_MagY      ;
	float Imu_MagZ      ;
	float Imu_Rot[9]    ;
	float Px4_Xdot      ;
	float Px4_Ydot      ;
	float Px4_Qual      ;
	float Lidar_Dist    ;
	float Camera_X      ;
	float Camera_Y      ;
	float Camera_Yaw    ;
	float Camera_T      ;
	float Filter_X      ;
	float Filter_Y      ;
	float Filter_Z      ;
	float Filter_Xdot   ;
	float Filter_Ydot   ;
	float Filter_Zdot   ;
	float Filter_Roll   ;
	float Filter_Pitch  ;
	float Filter_Yaw    ;
	float Val_P_X       ;
	float Val_I_X       ;
	float Val_D_X       ;
	float Val_P_Y       ;
	float Val_I_Y       ;
	float Val_D_Y       ;
	float Val_P_Z       ;
	float Val_I_Z       ;
	float Val_D_Z       ;
	float Val_P_Yaw     ;
	float Val_I_Yaw     ;
	float Val_D_Yaw     ;
	float Rate_P_X      ;
	float Rate_I_X      ;
	float Rate_D_X      ;
	float Rate_P_Y      ;
	float Rate_I_Y      ;
	float Rate_D_Y      ;
	float Rate_P_Z      ;
	float Rate_I_Z      ;
	float Rate_D_Z      ;
	float Rate_P_Yaw    ;
	float Rate_I_Yaw    ;
	float Rate_D_Yaw    ;
	float Setpt_X       ;
	float Setpt_Y       ;
	float Setpt_Z       ;
	float Setpt_Yaw     ;
	float Setpt_Xdot    ;
	float Setpt_Ydot    ;
	float Setpt_Zdot    ;
	float PID_Ux        ;
	float PID_Uy        ;
	float PID_Uz        ;
	float PID_Yawdot    ;
	float Flag_Xval     ;
	float Flag_Yval     ;
	float Flag_Zval     ;
	float Flag_Xrate    ;
	float Flag_Yrate    ;
	float Flag_Zrate    ;
	float Flag_Yawl     ;
	float Battery       ;
	float DJI_Roll      ;
	float DJI_Pitch     ;
	float DJI_Yawdot    ;
	float DJI_Fz        ;
	float AtomFlag      ;
	float DJI_Roll_RAW  ;
	float DJI_Pitch_RAW ;
	float DJI_Fz_RAW    ;
	float DJI_YawRate_RAW;
	float PPM_1;
	float PPM_2;
	float PPM_3;
	float PPM_4;              
	float msTime;

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
	// Fz, R, P, Ydot
	float ctrlInData[4] = {0, 0, 0, 0};
	arm_matrix_instance_f32 ctrlIn;
	arm_mat_init_f32(&ctrlIn, numCtrlInputs, 1, ctrlInData);

	// set up y column vector of sensor measurements
	// Z, Xdot, Ydot
	float sensorsNoCamData[3] = {0, 0, 0};
	arm_matrix_instance_f32 sensorsNoCam;
	arm_mat_init_f32(&sensorsNoCam, numSensorsNoCam, 1, sensorsNoCamData);
	
	float dt = 0.010f;
	float mass = 2.50f;
	float gravity = 9.81f;
	arm_matrix_instance_f32 x;
	while(	//read data from log
		logfile >>
			Time          >>
			Mode          >>
			Imu_AccX      >>
			Imu_AccY      >>
			Imu_AccZ      >>
			Imu_AngRateX  >>
			Imu_AngRateY  >>
			Imu_AngRateZ  >>
			Imu_MagX      >>
			Imu_MagY      >>
			Imu_MagZ      >>
			Imu_Rot[0]      >>
			Imu_Rot[1]      >>
			Imu_Rot[2]      >>
			Imu_Rot[3]      >>
			Imu_Rot[4]      >>
			Imu_Rot[5]      >>
			Imu_Rot[6]      >>
			Imu_Rot[7]      >>
			Imu_Rot[8]      >>
			Px4_Xdot      >>
			Px4_Ydot      >>
			Px4_Qual      >>
			Lidar_Dist    >>
			Camera_X      >>
			Camera_Y      >>
			Camera_Yaw    >>
			Camera_T      >>
			Filter_X      >>
			Filter_Y      >>
			Filter_Z      >>
			Filter_Xdot   >>
			Filter_Ydot   >>
			Filter_Zdot   >>
			Filter_Roll   >>
			Filter_Pitch  >>
			Filter_Yaw    >>
			Val_P_X       >>
			Val_I_X       >>
			Val_D_X       >>
			Val_P_Y       >>
			Val_I_Y       >>
			Val_D_Y       >>
			Val_P_Z       >>
			Val_I_Z       >>
			Val_D_Z       >>
			Val_P_Yaw     >>
			Val_I_Yaw     >>
			Val_D_Yaw     >>
			Rate_P_X      >>
			Rate_I_X      >>
			Rate_D_X      >>
			Rate_P_Y      >>
			Rate_I_Y      >>
			Rate_D_Y      >>
			Rate_P_Z      >>
			Rate_I_Z      >>
			Rate_D_Z      >>
			Rate_P_Yaw    >>
			Rate_I_Yaw    >>
			Rate_D_Yaw    >>
			Setpt_X       >>
			Setpt_Y       >>
			Setpt_Z       >>
			Setpt_Yaw     >>
			Setpt_Xdot    >>
			Setpt_Ydot    >>
			Setpt_Zdot    >>
			PID_Ux        >>
			PID_Uy        >>
			PID_Uz        >>
			PID_Yawdot    >>
			Flag_Xval     >>
			Flag_Yval     >>
			Flag_Zval     >>
			Flag_Xrate    >>
			Flag_Yrate    >>
			Flag_Zrate    >>
			Flag_Yawl     >>
			Battery       >>
			DJI_Roll      >>
			DJI_Pitch     >>
			DJI_Yawdot    >>
			DJI_Fz        >>
			AtomFlag      >>
			DJI_Roll_RAW  >>
			DJI_Pitch_RAW >>
			DJI_Fz_RAW    >>
			DJI_YawRate_RAW >>
			PPM_1 >>
			PPM_2 >>
			PPM_3 >>
			PPM_4 >> 
			msTime)
	{
		//usePredict from control runnable
		if((Time -lastTime) < 0.015) 
		{
			usePredict = true;
		}
		else 
		{
			usePredict = false;
		}
		lastTime = Time;

		// run prediction if usePredict
		ctrlIn.pData[0] = (mass * gravity) + DJI_Fz_RAW; //Fz Control Input
		//ctrlIn.pData[0] = 0;
		ctrlIn.pData[1] = DJI_Roll_RAW; //Roll Control Input
		ctrlIn.pData[2] = DJI_Pitch_RAW; //Pitch Control Input
		ctrlIn.pData[3] = DJI_YawRate_RAW; //Yaw Control Input
		float rollSin = float(sin(atan2(Imu_Rot[5], Imu_Rot[8])));
		float rollCos = float(cos(atan2(Imu_Rot[5], Imu_Rot[8])));
		float pitchSin = float(-Imu_Rot[2]);
		float pitchCos = float(cos(asin(-Imu_Rot[2])));
		float yawSin = float(sin(atan2(Imu_Rot[1], Imu_Rot[0]))); //refYaw is missing
		float yawCos = float(cos(atan2(Imu_Rot[1], Imu_Rot[0])));
		if(usePredict) //check usePredict otherwise no prediction
		{
			ekf.predict(dt, mass, rollSin, rollCos, pitchSin, 
				pitchCos, yawSin, yawCos, &ctrlIn);
			x = ekf.getState();
		
			cout << "X after Prediction with dt = " << dt << 
				" and u1 = " << ctrlIn.pData[0] << endl;
			for (uint16_t i = 0; i < x.numRows; ++i) 
				cout << x.pData[i] << " ";
			cout << endl;
		}
		
		// run correction
		sensorsNoCam.pData[0] = Lidar_Dist; //Z
		sensorsNoCam.pData[1] = Px4_Xdot; //Xdot
		sensorsNoCam.pData[2] = Px4_Ydot; //Ydot
		if(!usePredict) //check usePredict otherwise no correction
		{
			ekf.update(dt, noCamId, &sensorsNoCam);
			x = ekf.getState();
		
			cout << "Sensor input [z, xdot, ydot] = ";
			for (uint16_t i = 0; i < sensorsNoCam.numRows; ++i) 
				cout << sensorsNoCam.pData[i] << " ";
			cout << endl;
		
			cout << "X after Correction = " << endl;
			for (uint16_t i = 0; i < x.numRows; ++i) 
				cout << x.pData[i] << " ";
			cout << endl;
		}

		// log output to file
		outfile << Time << "\t";
		outfile << Lidar_Dist << "\t";
		for (uint16_t i = 0; i < x.numRows; ++i) outfile << x.pData[i] << "\t";
		outfile << endl;
	}
	outfile.close();
	logfile.close();
	return 0;
}

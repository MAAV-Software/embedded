#include <iostream>
#include <fstream>
#include <stdint.h>
#include <math.h>
#include "cmeigen.hpp"
#include "Vehicle.hpp"
#include "MaavMath.hpp"

using namespace MaavMath;
using namespace std;

int main(int argc, char **argv)
{
	if(argc < 2)
	{
		cout << "logkalman logfile outfile" << endl;
	}
	ifstream logfile(argv[1]);//open first parameter as log file
	ofstream outfile(argv[2]);//open second parameter as output file
	if(!logfile) return 1; //could not open

	//Fake PID gains to instantiate Vehicle
	float valueGains[NUM_DOFS][NUM_PID_GAINS];
	float rateGains[NUM_DOFS][NUM_PID_GAINS];
	for(int i = 0; i < NUM_DOFS; ++i)
	{
		for(int j = 0; j < NUM_PID_GAINS; ++j)
		{
			valueGains[i][j] = 0;
			rateGains[i][j] = 0;
		}
	}

	//Dummy to instantiate Vehicle
	PidLog plogs[NUM_DOFS][2];

	//To extract log data from vehicle
	VehicleLog vlog; 
	
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
	float lidarTime;
	float px4Time;
	float imuTime;
	float poseTime;
	float refYaw;

	Vehicle v(valueGains, rateGains);

	while(	//read data from log
		logfile >>
			Time            >>
			Mode            >>
			Imu_AccX        >>
			Imu_AccY        >>
			Imu_AccZ        >>
			Imu_AngRateX    >>
			Imu_AngRateY    >>
			Imu_AngRateZ    >>
			Imu_MagX        >>
			Imu_MagY        >>
			Imu_MagZ        >>
			Imu_Rot[0]      >>
			Imu_Rot[1]      >>
			Imu_Rot[2]      >>
			Imu_Rot[3]      >>
			Imu_Rot[4]      >>
			Imu_Rot[5]      >>
			Imu_Rot[6]      >>
			Imu_Rot[7]      >>
			Imu_Rot[8]      >>
			Px4_Xdot        >>
			Px4_Ydot        >>
			Px4_Qual        >>
			Lidar_Dist      >>
			Camera_X        >>
			Camera_Y        >>
			Camera_Yaw      >>
			Camera_T        >>
			Filter_X        >>
			Filter_Y        >>
			Filter_Z        >>
			Filter_Xdot     >>
			Filter_Ydot     >>
			Filter_Zdot     >>
			Filter_Roll     >>
			Filter_Pitch    >>
			Filter_Yaw      >>
			Val_P_X         >>
			Val_I_X         >>
			Val_D_X         >>
			Val_P_Y         >>
			Val_I_Y         >>
			Val_D_Y         >>
			Val_P_Z         >>
			Val_I_Z         >>
			Val_D_Z         >>
			Val_P_Yaw       >>
			Val_I_Yaw       >>
			Val_D_Yaw       >>
			Rate_P_X        >>
			Rate_I_X        >>
			Rate_D_X        >>
			Rate_P_Y        >>
			Rate_I_Y        >>
			Rate_D_Y        >>
			Rate_P_Z        >>
			Rate_I_Z        >>
			Rate_D_Z        >>
			Rate_P_Yaw      >>
			Rate_I_Yaw      >>
			Rate_D_Yaw      >>
			Setpt_X         >>
			Setpt_Y         >>
			Setpt_Z         >>
			Setpt_Yaw       >>
			Setpt_Xdot      >>
			Setpt_Ydot      >>
			Setpt_Zdot      >>
			PID_Ux          >>
			PID_Uy          >>
			PID_Uz          >>
			PID_Yawdot      >>
			Flag_Xval       >>
			Flag_Yval       >>
			Flag_Zval       >>
			Flag_Xrate      >>
			Flag_Yrate      >>
			Flag_Zrate      >>
			Flag_Yawl       >>
			Battery         >>
			DJI_Roll        >>
			DJI_Pitch       >>
			DJI_Yawdot      >>
			DJI_Fz          >>
			AtomFlag        >>
			DJI_Roll_RAW    >>
			DJI_Pitch_RAW   >>
			DJI_Fz_RAW      >>
			DJI_YawRate_RAW >>
			PPM_1           >>
			PPM_2           >>
			PPM_3           >>
			PPM_4           >> 
			msTime          >> 
			lidarTime       >>
			px4Time         >>
			imuTime         >>
			poseTime        >>
			refYaw
	)
	{
		float yaw = atan2(Imu_Rot[1], Imu_Rot[0]);
		v.runFilter(Imu_Rot, yaw,
			Imu_AccX, Imu_AccY, Imu_AccZ, Time,
			Lidar_Dist, lidarTime,
			Px4_Xdot, Px4_Ydot, px4Time, 
			0, 0, 0);

		//log the state estimate
		v.prepareLog(vlog, plogs);
		outfile <<
			vlog.xFilt << "\t" <<
			vlog.xdotFilt << "\t" <<
			vlog.yFilt << "\t" <<
			vlog.ydotFilt << "\t" <<
			vlog.zFilt << "\t" <<
			vlog.zdotFilt << endl;
	}
	outfile.close();
	logfile.close();
	return 0;
}

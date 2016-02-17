#define BOOST_TEST_MODULE "VehicleRunFilterTest"
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include <iostream>
#include <fstream>
#include <cmath>
#include <stdint.h>

#include "cmeigen.hpp"
#include "Vehicle.hpp"
#include "MaavMath.hpp"
#include "Vehicle.hpp"
#include "Dof.hpp"

using namespace std;

struct Fixture
{	
	const float qx;
	const float qxd;
	const float qy;
	const float qyd;
	const float qz;
	const float qzd;
	const float rlidarz;
	const float rlidarzd;
	const float rpx4xd;
	const float rpx4yd;

	float valueGains[NUM_DOFS][NUM_PID_GAINS];
	float rateGains[NUM_DOFS][NUM_PID_GAINS];
	
	Vehicle *v1;
	PidLog plog[NUM_DOFS][2];
	VehicleLog vlog;
	~Fixture()
	{
		delete v1;
	}
	Fixture() :
//SELECT Q AND R MATRIX VALUES HERE============================================
		qx (0.5),
		qxd(0.1),
		qy (0.5),
		qyd(0.1),
		qz (0.1),
		qzd(0.1),
		rlidarz (0.05),
		rlidarzd(0.05),
		rpx4xd(0.8),
		rpx4yd(0.8)
	{	
		//Dummy PID Gains for Vehicle Class
		for(int i = 0; i < NUM_DOFS; ++i)
		{
			for(int j = 0; j < NUM_PID_GAINS; ++j)
			{
				valueGains[i][j] = 0;
				rateGains[i][j] = 0;
			}
		}
	
		//Instantiate new Vehicle
		v1 = new Vehicle(valueGains, rateGains);
		v1->setQR(qx, qxd, qy, qyd, qz, qzd, 
			rlidarz, rlidarzd, 
			rpx4xd, rpx4yd, 
			0.0, 0.0);
	}

};

BOOST_AUTO_TEST_CASE(RunFilterRealDataTest)
{
	//Fixture
	Fixture f;

	//Log File to test with
	ifstream logfile("RunFilterTestLog.TXT");
	
	//Log Reading Variables
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

	float correct[5][6] = 
    {
		{2.1140, -0.7781, 2.0910, -0.7696, -0.0015, 0.0351},
		{2.0868, -0.7829, 2.0640, -0.7743, -0.0005, 0.0184},
		{2.0758, -0.7847, 2.0532, -0.7763, -0.0002, 0.0194},
		{2.4037, -0.4501, 2.3777, -0.4454,  0.0001, 0.0205},
		{2.3965, -0.4476, 2.3705, -0.4476,  0.0000, 0.0135}
	};

	//Check initial state
	f.v1->prepareLog(f.vlog, f.plog);
	BOOST_CHECK(abs(f.vlog.xFilt)    < 0.0001);
	BOOST_CHECK(abs(f.vlog.yFilt)    < 0.0001);
	BOOST_CHECK(abs(f.vlog.zFilt)    < 0.0001);
	BOOST_CHECK(abs(f.vlog.xdotFilt) < 0.0001);
	BOOST_CHECK(abs(f.vlog.ydotFilt) < 0.0001);
	BOOST_CHECK(abs(f.vlog.zdotFilt) < 0.0001);

	//Run the filter five iterations
	for(int i = 0; i < 6; ++i)
	{
		//Read out of the logfile
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
			refYaw;

		//Run the filter
		float yaw = atan2(Imu_Rot[1], Imu_Rot[0]);
		f.v1->runFilter(Imu_Rot, yaw,
			Imu_AccX, Imu_AccY, Imu_AccZ, Time,
			Lidar_Dist, lidarTime,
			Px4_Xdot, Px4_Ydot, px4Time, 
			0, 0, 0);

		//Compare the output
		f.v1->prepareLog(f.vlog, f.plog);
    
        cout << "\nITER " << i << "\n";
		BOOST_CHECK(abs(f.vlog.xFilt - correct[i][0])    < 0.001);
		BOOST_CHECK(abs(f.vlog.xdotFilt - correct[i][1]) < 0.001);
		BOOST_CHECK(abs(f.vlog.yFilt - correct[i][2])    < 0.001);
		BOOST_CHECK(abs(f.vlog.ydotFilt - correct[i][3]) < 0.001);
		BOOST_CHECK(abs(f.vlog.zFilt - correct[i][4])    < 0.001);
		BOOST_CHECK(abs(f.vlog.zdotFilt - correct[i][5]) < 0.001);
	}
}

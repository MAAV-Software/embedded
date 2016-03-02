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
    BOOST_CHECK(logfile);
    
    ifstream answerFile("runFltAns.txt");
    BOOST_CHECK(answerFile);

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
    
    float correct[6];

    /*
    float correct[6][6] = 
    {
        {0.000000,   0.000000,  -0.000100,  -0.000300,  -0.000400,  -0.000600}, 
        {0.000000,  -0.003800,  -0.008700,  -0.010500,  -0.012300,  -0.014500}, 
        {0.000000,   0.000000,  -0.000100,  -0.000300,  -0.000400,  -0.000600}, 
        {0.000000,  -0.003800,  -0.008600,  -0.010500,  -0.012500,  -0.014700}, 
        {0.000000,   0.000000,   0.000100,   0.000100,   0.000200,   0.000200}, 
        {0.000000,   0.001900,   0.003900,   0.004900,   0.006000,   0.006000}, 
    };
    
    */

	//Check initial state
	f.v1->prepareLog(f.vlog, f.plog);
	BOOST_CHECK(abs(f.vlog.xFilt)    < 0.0001);
	BOOST_CHECK(abs(f.vlog.yFilt)    < 0.0001);
	BOOST_CHECK(abs(f.vlog.zFilt)    < 0.0001);
	BOOST_CHECK(abs(f.vlog.xdotFilt) < 0.0001);
	BOOST_CHECK(abs(f.vlog.ydotFilt) < 0.0001);
	BOOST_CHECK(abs(f.vlog.zdotFilt) < 0.0001);

	//Run the filter five iterations
	for(int i = 0; i < 4847; ++i)
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

        for (size_t j = 0; j < 6; ++j) answerFile >> correct[j];

		//Run the filter
		float yaw = atan2(Imu_Rot[1], Imu_Rot[0]);
		f.v1->runFilter(Imu_Rot, yaw,
			Imu_AccX, Imu_AccY, Imu_AccZ, Time,
			Lidar_Dist, lidarTime,
			Px4_Xdot, Px4_Ydot, px4Time, 
			Camera_X, Camera_Y, Camera_T);

		//Compare the output
		f.v1->prepareLog(f.vlog, f.plog);
        
		if (abs(f.vlog.xFilt -    correct[0]) >= 0.0010) cout << "Iter: " << i << "\n"; 
		if (abs(f.vlog.xdotFilt - correct[1]) >= 0.0015) cout << "Iter: " << i << "\n";
		if (abs(f.vlog.yFilt -    correct[2]) >= 0.0010) cout << "Iter: " << i << "\n";
		if (abs(f.vlog.ydotFilt - correct[3]) >= 0.0015) cout << "Iter: " << i << "\n";
		if (abs(f.vlog.zFilt -    correct[4]) >= 0.0010) cout << "Iter: " << i << "\n";
		if (abs(f.vlog.zdotFilt - correct[5]) >= 0.0015) 
        {
            cout << "Iter: " << i << "\n";
            cout << f.vlog.zdotFilt << "\n" << correct[5] << "\n";
            cout << abs(f.vlog.zdotFilt - correct[5]) << "\n";
        }   
        
		BOOST_CHECK(abs(f.vlog.xFilt -    correct[0]) < 0.0010);
		BOOST_CHECK(abs(f.vlog.xdotFilt - correct[1]) < 0.0015);
		BOOST_CHECK(abs(f.vlog.yFilt -    correct[2]) < 0.0010);
		BOOST_CHECK(abs(f.vlog.ydotFilt - correct[3]) < 0.0015);
		BOOST_CHECK(abs(f.vlog.zFilt -    correct[4]) < 0.0010);
		BOOST_CHECK(abs(f.vlog.zdotFilt - correct[5]) < 0.0015);
    /*    
        cout << "\nITER " << i << " Time " << Time << " \n";
        cout << f.vlog.xFilt << " " << f.vlog.xdotFilt << " ";
        cout << f.vlog.yFilt << " " << f.vlog.ydotFilt << " ";
        cout << f.vlog.zFilt << " " << f.vlog.zdotFilt << "\n";
        cout << correct[0][i] << " " << correct[1][i] << " ";
        cout << correct[2][i] << " " << correct[3][i] << " ";
        cout << correct[4][i] << " " << correct[5][i] << "\n";
        
		BOOST_CHECK(abs(f.vlog.xFilt -    correct[0][i]) < 0.001);
	    BOOST_CHECK(abs(f.vlog.xdotFilt - correct[1][i]) < 0.001);
		BOOST_CHECK(abs(f.vlog.yFilt -    correct[2][i]) < 0.001);
		BOOST_CHECK(abs(f.vlog.ydotFilt - correct[3][i]) < 0.001);
		BOOST_CHECK(abs(f.vlog.zFilt -    correct[4][i]) < 0.001);
		BOOST_CHECK(abs(f.vlog.zdotFilt - correct[5][i]) < 0.001);
    */
	}
    
    answerFile.close();
    logfile.close();
}

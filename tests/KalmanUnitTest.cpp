#define BOOST_TEST_MODULE "KalmanFilterTest"
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <fstream>
#include <string>

#include "kalman/KalmanFilter.hpp"
#include "cmeigen.hpp"
#include "MaavMath.hpp"

using namespace std;

struct Fixture
{
	KalmanFilter kf;
	
	// sensor feedback (or ctrl input in Imu's case)
	float xcam;
	float ycam;
	float xdotPx4;
	float ydotPx4;
	float zLidar;
	float zdotLidar;
	float xddotImu;
	float yddotImu;
	float zddotImu;
	float dt;
	
	Fixture()
	{
		dt = 0.010;
		xcam = 5.7;
		ycam = 15.5;
		xdotPx4 = 0.5;
		ydotPx4 = 1.0;
		zLidar = 1.5;
		zdotLidar = 0.01;
		xddotImu = 0.01;
		yddotImu = 1.0;
		zddotImu = 0.001;
	}
};

BOOST_AUTO_TEST_CASE(ctorTest)
{
	Fixture f;
	const arm_matrix_instance_f32& x = f.kf.getState();
	const arm_matrix_instance_f32& P = f.kf.getCovar();
  
    for (uint16_t r = 0; r < x.numRows; ++r)
    {
        for (uint16_t c = 0; c < x.numCols; ++c)
            BOOST_CHECK_CLOSE(MaavMath::mat_at(x, r, c), 0.0, 0.0001);
    }
    
    for (uint16_t r = 0; r < P.numRows; ++r)
    {
        for (uint16_t c = 0; c < P.numCols; ++c)
            BOOST_CHECK_CLOSE(MaavMath::mat_at(P, r, c), 0.0, 0.0001);
    }
}


BOOST_AUTO_TEST_CASE(ptest)
{
	Fixture f;
    f.kf.predict(f.xddotImu, f.yddotImu, f.zddotImu, f.dt); 
	const arm_matrix_instance_f32& x = f.kf.getState();
    const arm_matrix_instance_f32& P = f.kf.getCovar();

    BOOST_CHECK_CLOSE(MaavMath::mat_at(x, 0, 0), 0.00000, 100);
    BOOST_CHECK_CLOSE(MaavMath::mat_at(x, 1, 0), 0.00010, 100);
    BOOST_CHECK_CLOSE(MaavMath::mat_at(x, 2, 0), 0.00000, 100);
    BOOST_CHECK_CLOSE(MaavMath::mat_at(x, 3, 0), 0.01000, 100);
    BOOST_CHECK_CLOSE(MaavMath::mat_at(x, 4, 0), 0.00000, 100);
    BOOST_CHECK_CLOSE(MaavMath::mat_at(x, 5, 0), 0.00001, 100);
  
    for (uint16_t r = 0; r < P.numRows; ++r)
    { 
        for (uint16_t c = 0; c < P.numCols; ++c)
            BOOST_CHECK_CLOSE(MaavMath::mat_at(P, r, c), 0.0, 0.0001);
    }
}

BOOST_AUTO_TEST_CASE(resetTest)
{
	Fixture f;
    f.kf.predict(f.xddotImu, f.yddotImu, f.zddotImu, f.dt);    
	f.kf.reset();
	const arm_matrix_instance_f32& x = f.kf.getState();
	const arm_matrix_instance_f32& P = f.kf.getCovar();
    
    for (uint16_t r = 0; r < x.numRows; ++r)
    {
        for (uint16_t c = 0; c < x.numCols; ++c)
            BOOST_CHECK_CLOSE(MaavMath::mat_at(x, r, c), 0.0, 0.0001);
    }
    
    for (uint16_t r = 0; r < P.numRows; ++r)
    {
        for (uint16_t c = 0; c < P.numCols; ++c)
            BOOST_CHECK_CLOSE(MaavMath::mat_at(P, r, c), 0.0, 0.0001);
    }
}

BOOST_AUTO_TEST_CASE(correctPx4Test)
{
	Fixture f;

    f.kf.setQ(.5, .9, .7, .2, .1, 1.0);//same as those used for Matlab testing
    f.kf.setR_Px4(.1, .55);

    f.kf.predict(f.xddotImu, f.yddotImu, f.zddotImu, f.dt);    
    f.kf.correctPx4(f.xdotPx4, f.ydotPx4);

    const arm_matrix_instance_f32& testState = f.kf.getState();
    const arm_matrix_instance_f32& testCovar = f.kf.getCovar();

    float ans;

    ifstream answerFile("Px4Correct.txt");
    
    BOOST_CHECK(answerFile.is_open());

    for(int i = 0; i < testState.numRows; ++i)
    {
        for(int j = 0; j < testState.numCols; ++j)
        {
            if(answerFile >> ans) 
            {
                cout << "Should be: " << ans << ". Actual: " << 
                    MaavMath::mat_at(testState, i, j) << endl;
                BOOST_CHECK_CLOSE(MaavMath::mat_at(testState, i, j), ans, 1.0);
            }
        }
    }

    for(int i = 0; i < testCovar.numRows; ++i)
    {
        for(int j = 0; j < testCovar.numCols; ++j)
        {
            if(answerFile >> ans) 
            {
                cout << "Should be: " << ans << ". Actual: " << 
                    MaavMath::mat_at(testCovar, i, j) << endl;
                BOOST_CHECK_CLOSE(MaavMath::mat_at(testCovar, i, j), ans, 1.0);
            }
        }
    }

    //answerfile should no longer be open
    answerFile.close();

    BOOST_CHECK( !(answerFile.is_open()) );
}

/*
BOOST_AUTO_TEST_CASE(correctLidarTest)
{
	Fixture f;

	const arm_matrix_instance_f32& x = f.kf.getState();
	const arm_matrix_instance_f32& P = f.kf.getCovar();
}

BOOST_AUTO_TEST_CASE(correctCameraTest)
{
	Fixture f;

	const arm_matrix_instance_f32& x = f.kf.getState();
	const arm_matrix_instance_f32& P = f.kf.getCovar();
}
*/

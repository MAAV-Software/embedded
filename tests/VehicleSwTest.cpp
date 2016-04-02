//#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "VehicleSoftwareTest"
#define PI 3.14159265358979f
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include <stdint.h>
#include "Vehicle.hpp"
#include "Dof.hpp"

#include <iostream>

void diagonalMatIint(float *mat, int dim, float *diag)
{
	for(int i = 0; i < dim; ++i)
	{
		for(int j = 0; j < dim; ++j)	
		{
			if(i == j)
				mat[i*dim + j] = diag[i];
			else
				mat[i*dim + j] = 0.0f;
		}
	}
}

struct Fixture
{
	float states[NUM_DOFS][NUM_DOF_STATES];
	float setpts[NUM_DOFS][NUM_DOF_STATES];
	float setpts2[NUM_DOFS][NUM_DOF_STATES];
	float valueGains[NUM_DOFS][NUM_PID_GAINS];
	float rateGains[NUM_DOFS][NUM_PID_GAINS];
	uint8_t valueFlags[NUM_DOFS];
	uint8_t rateFlags[NUM_DOFS];
	float inertias[NUM_DOFS];
	float stateBounds[NUM_DOFS];
	float rateUpLims[NUM_DOFS];
	float rateLwLims[NUM_DOFS];
	float accelUpLims[NUM_DOFS];
	float accelLwLims[NUM_DOFS];
	float valueStateLpCoeffs[NUM_DOFS];
	float valueErrorLpCoeffs[NUM_DOFS];
	float rateStateLpCoeffs[NUM_DOFS];
	float rateErrorLpCoeffs[NUM_DOFS];
	float totalMass;
	float initTime;
	float rpLims[NUM_ANGLES];
	float ekfInitState[6];
	float ekfInitP[36];
	float ekfQ[36];
	float ekfNoCamR[9];
	float ekfWithCamR[25];

	/*
	ekfInitState[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
	ekfInitP[81];
	float ekfQ[81];
	float ekfNoCamR[36];
	float ekfWithCamR[64]);
	*/

	FlightMode mode;

	
	Vehicle *v1;
	PidLog plog[NUM_DOFS][2];
	VehicleLog vlog;
	~Fixture()
	{
		delete v1;
	}
	Fixture()
	{
		int i = 0, j = 0.00f;
		setpts[0][0] = 0.01f;
		setpts[0][1] = 0.02f;
		setpts[0][2] = 0.03f;
		setpts[0][3] = 0.03f;
		setpts[1][0] = 0.03f;
		setpts[1][1] = 0.04f;
		setpts[1][2] = 0.09f;
		setpts[1][3] = 0.09f;
		setpts[2][0] = 0.05f;
		setpts[2][1] = 0.06f;
		setpts[2][2] = 0.03f;
		setpts[2][3] = 0.03f;
		setpts[3][0] = 0.07f;
		setpts[3][1] = 0.08f;
		setpts[3][2] = 0.01f;
		setpts[3][3] = 0.02f;
		for (i = 0; i < NUM_DOFS; i++)
		{
			for(j = 0; j < NUM_DOF_STATES; j++)
			{
				states[i][j] = 0;
				setpts2[i][j] = setpts[i][j];
			}
		}
		setpts2[3][0] = 3*PI;
		/* above means:
		states[NUM_DOFS][NUM_DOF_STATES] =
			{{0, 0, 0, 0},
			 {0, 0, 0, 0}, 
			 {0, 0, 0, 0}, 
			 {0, 0, 0, 0},};
		setpts[NUM_DOFS][NUM_DOF_STATES] =
			{{0 01, 0 02, *, *}, 
			 {0.03, 0.04, *, *}, 
			 {0.05, 0.06, *, *},
			 {0.07, 0.08, *, *},};
		setpts2[NUM_DOFS][NUM_DOF_STATES] =
			{{1, 1, 1, 1}, 
			 {2, 2, 2, 2},
			 {3, 3, 3, 3},
			 {4, 4, 4, 4},};
		*/
		for (i = 0; i < NUM_DOFS; i++)
		{
			for(j = 0; j < NUM_DOFS; j++)
			{
				valueGains[i][j] = 6.0f * (float)i + (float)j;
				rateGains[i][j] = 6.0f * (float)i + (float)j + 3.0f;
			}
		}
		//std::cout<<rateGains[0][0]<<'\n';
		/*above means:
		valueGains[NUM_DOFS][NUM_PID_GAINS] =
			{{0, 1, 2},
			 {6, 7, 8}, 
			 {12, 13, 14},
			 {18, 19, 20},};
		rateGains[NUM_DOFS][NUM_PID_GAINS] =
			{{3, 4, 5}, 
			 {9, 10, 11}, 
			 {15, 16, 17},
			 {21, 22, 23},};
		*/
		//for (i = 0; )
		valueFlags[0] = 0;
		valueFlags[1] = 0;
		valueFlags[2] = 0;
		valueFlags[3] = DERR_DT_MASK | DISC_DERIV_MASK | WRAP_AROUND_MASK;
		rateFlags[0] = DERR_DT_MASK | DISC_DERIV_MASK;
		rateFlags[1] = DERR_DT_MASK | DISC_DERIV_MASK;
		rateFlags[2] = DERR_DT_MASK | DISC_DERIV_MASK;
		rateFlags[3] = DERR_DT_MASK | DISC_DERIV_MASK;
		/*above means
		uint8_t valueFlags[NUM_DOFS] = {
			DERR_DT_MASK,
			DERR_DT_MASK,
			DERR_DT_MASK,
			DERR_DT_MASK | DISC_DERIV_MASK | WRAP_AROUND_MASK,
		};
		uint8_t rateFlags[NUM_DOFS] = {
			DERR_DT_MASK | DISC_DERIV_MASK,
			DERR_DT_MASK | DISC_DERIV_MASK,
			DERR_DT_MASK | DISC_DERIV_MASK,
			DERR_DT_MASK | DISC_DERIV_MASK,
		};
		*/
		for (i = 0; i < 4; ++i)
		{
			//valueFlags[i] = 0.0f;
			//rateFlags[i] = 0.0f;
			inertias[i] = 0.0f;
			stateBounds[i] = 0.0f;
			rateUpLims[i] = 0.0f;
			rateLwLims[i] = 0.0f;
			accelUpLims[i] = 0.0f;
			accelLwLims[i] = 0.0f;
			valueStateLpCoeffs[i] = 0.0f;
			valueErrorLpCoeffs[i] = 0.0f;
			rateStateLpCoeffs[i] = 0.0f;
			rateErrorLpCoeffs[i] = 0.0f;
		}
		totalMass = 0.0f;
		initTime = 0.0f;
		rpLims[0] = 0.0f;
		rpLims[1] = 0.0f;
		ekfInitState[0] = 0.0f;
		ekfInitState[1] = 0.0f;
		ekfInitState[2] = 0.0f;
		ekfInitState[3] = 0.0f;
		ekfInitState[4] = 0.0f;
		ekfInitState[5] = 0.0f;
		float ekfInitP_diag[6] = { 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f };
		float ekfQ_diag[6] = { 0.1f, 0.1f, 0.9f, 0.01f, 0.01f, 0.01f };
		float ekfNoCamR_diag[3] = { 0.02f, 0.1f, 0.1f };
		float ekfWithCamR_diag[5] = { 0.1f, 0.1f, 0.02f, 0.1f, 0.1f };
		diagonalMatIint(ekfInitP, 6, ekfInitP_diag);
		diagonalMatIint(ekfQ, 6, ekfQ_diag);
		diagonalMatIint(ekfNoCamR, 3, ekfNoCamR_diag);
		diagonalMatIint(ekfWithCamR, 5, ekfWithCamR_diag);
		/*above means:
		ekfInitP[36] = {
			0.1, 0, 0, 0, 0, 0,
			0, 0.1, 0, 0, 0, 0,
			0, 0, 0.1, 0, 0, 0,
			0, 0, 0, 0.1, 0, 0,
			0, 0, 0, 0, 0.1, 0,
			0, 0, 0, 0, 0, 0.1
		};
		ekfQ[36] = {
			0.1, 0, 0, 0, 0, 0,
			0, 0.1, 0, 0, 0, 0,
			0, 0, 0.9, 0, 0, 0,
			0, 0, 0, 0.01, 0, 0,
			0, 0, 0, 0, 0.01, 0,
			0, 0, 0, 0, 0, 0.01
		};
		ekfNoCamR[9] = {
			0.02, 0, 0,
			0, 0.1, 0,
			0, 0, 0.1
		};
		ekfWithCamR[25] = {
			0.1, 0, 0, 0, 0,
			0, 0.1, 0, 0, 0,
			0, 0, 0.02, 0, 0,
			0, 0, 0, 0.1, 0,
			0, 0, 0, 0, 0.1
		};*/
		mode = AUTONOMOUS;
		//mode = ASSISTED;
		v1 = new Vehicle(valueGains, rateGains);
	}

};

BOOST_AUTO_TEST_CASE(ctorTest)
{
	Fixture f;
	Dji temp = f.v1->getDjiVals();
	BOOST_CHECK_EQUAL(temp.roll, 0);
	BOOST_CHECK_EQUAL(temp.pitch, 0);
	BOOST_CHECK_EQUAL(temp.yawRate, 0);
	BOOST_CHECK_EQUAL(temp.thrust, 0);
	f.v1->prepareLog(f.vlog, f.plog);

	for (int i = 0; i < NUM_DOFS; i++)
	{
		BOOST_CHECK_CLOSE(f.plog[i][0].kp, f.valueGains[i][0], 0.01);
		BOOST_CHECK_CLOSE(f.plog[i][0].ki, f.valueGains[i][1], 0.01);
		BOOST_CHECK_CLOSE(f.plog[i][0].kd, f.valueGains[i][2], 0.01);
		BOOST_CHECK_CLOSE(f.plog[i][1].kp, f.rateGains[i][0], 0.01);
		BOOST_CHECK_CLOSE(f.plog[i][1].ki, f.rateGains[i][1], 0.01);
		BOOST_CHECK_CLOSE(f.plog[i][1].kd, f.rateGains[i][2], 0.01);	
		BOOST_CHECK_CLOSE(f.plog[i][0].setpt, (float)0, 0.01);
		BOOST_CHECK_CLOSE(f.plog[i][1].setpt, (float)0, 0.01);
		BOOST_CHECK_CLOSE(f.plog[i][0].flags, (float)f.valueFlags[i], 0.01);
		BOOST_CHECK_CLOSE(f.plog[i][1].flags, (float)f.rateFlags[i], 0.01);
	}
}


BOOST_AUTO_TEST_CASE(setptTest_AUTONOMOUS)
{
	Fixture f;
	f.v1->setSetpt(f.setpts, AUTONOMOUS, false);
	f.v1->prepareLog(f.vlog, f.plog);
	for (int i = 0; i < NUM_DOFS; i++)
	{
		BOOST_CHECK_CLOSE(f.plog[i][0].kp, f.valueGains[i][0], 0.01);
		BOOST_CHECK_CLOSE(f.plog[i][0].ki, f.valueGains[i][1], 0.01);
		BOOST_CHECK_CLOSE(f.plog[i][0].kd, f.valueGains[i][2], 0.01);
		BOOST_CHECK_CLOSE(f.plog[i][1].kp, f.rateGains[i][0], 0.01);
		BOOST_CHECK_CLOSE(f.plog[i][1].ki, f.rateGains[i][1], 0.01);
		BOOST_CHECK_CLOSE(f.plog[i][1].kd, f.rateGains[i][2], 0.01);
		BOOST_CHECK_CLOSE(f.plog[i][0].setpt, f.setpts[i][0], 0.01);
		BOOST_CHECK_CLOSE(f.plog[i][1].setpt, (float)0, 0.01);
		BOOST_CHECK_CLOSE(f.plog[i][0].flags, (float)f.valueFlags[i], 0.01);
		BOOST_CHECK_CLOSE(f.plog[i][1].flags, (float)f.rateFlags[i], 0.01);
	}
}

BOOST_AUTO_TEST_CASE(setptTest_ASSISTED)
{
	Fixture f;
	f.v1->setSetpt(f.setpts, ASSISTED, false);
	f.v1->prepareLog(f.vlog, f.plog);
	BOOST_CHECK_CLOSE(f.plog[0][0].setpt, (float)0, 0.01);
	BOOST_CHECK_CLOSE(f.plog[0][1].setpt, f.setpts[0][1], 0.01);
	BOOST_CHECK_CLOSE(f.plog[1][0].setpt, (float)0, 0.01);
	BOOST_CHECK_CLOSE(f.plog[1][1].setpt, f.setpts[1][1], 0.01);
	BOOST_CHECK_CLOSE(f.plog[2][0].setpt, f.setpts[2][0], 0.01);
	BOOST_CHECK_CLOSE(f.plog[2][1].setpt, (float)0, 0.01);
	BOOST_CHECK_CLOSE(f.plog[3][0].setpt, f.setpts[3][0], 0.01);
	BOOST_CHECK_CLOSE(f.plog[3][1].setpt, (float)0, 0.01);
	for (int i = 0; i < NUM_DOFS; i++)
	{
		BOOST_CHECK_CLOSE(f.plog[i][0].kp, f.valueGains[i][0], 0.01);
		BOOST_CHECK_CLOSE(f.plog[i][0].ki, f.valueGains[i][1], 0.01);
		BOOST_CHECK_CLOSE(f.plog[i][0].kd, f.valueGains[i][2], 0.01);
		BOOST_CHECK_CLOSE(f.plog[i][1].kp, f.rateGains[i][0], 0.01);
		BOOST_CHECK_CLOSE(f.plog[i][1].ki, f.rateGains[i][1], 0.01);
		BOOST_CHECK_CLOSE(f.plog[i][1].kd, f.rateGains[i][2], 0.01);
		BOOST_CHECK_CLOSE(f.plog[i][0].flags, (float)f.valueFlags[i], 0.01);
		BOOST_CHECK_CLOSE(f.plog[i][1].flags, (float)f.rateFlags[i], 0.01);
	}
}

BOOST_AUTO_TEST_CASE(setptTest_Bound)
{
	Fixture f;
	f.v1->setSetpt(f.setpts2, ASSISTED, false);
	f.v1->prepareLog(f.vlog, f.plog);
	BOOST_CHECK_CLOSE(f.plog[3][0].setpt, (float)PI, 0.01);
}

BOOST_AUTO_TEST_CASE(setgTest)
{
	Fixture f;
	f.v1->setGains(f.valueGains, f.rateGains);
	f.v1->prepareLog(f.vlog, f.plog);
	for (int i = 0; i < NUM_DOFS; i++)
	{
		BOOST_CHECK_CLOSE(f.plog[i][0].kp, f.valueGains[i][0], 0.01);
		BOOST_CHECK_CLOSE(f.plog[i][0].ki, f.valueGains[i][1], 0.01);
		BOOST_CHECK_CLOSE(f.plog[i][0].kd, f.valueGains[i][2], 0.01);
		BOOST_CHECK_CLOSE(f.plog[i][1].kp, f.rateGains[i][0], 0.01);
		BOOST_CHECK_CLOSE(f.plog[i][1].ki, f.rateGains[i][1], 0.01);
		BOOST_CHECK_CLOSE(f.plog[i][1].kd, f.rateGains[i][2], 0.01);
		BOOST_CHECK_CLOSE(f.plog[i][0].setpt, (float)0, 0.01);
		BOOST_CHECK_CLOSE(f.plog[i][1].setpt, (float)0, 0.01);
		BOOST_CHECK_CLOSE(f.plog[i][0].flags, (float)f.valueFlags[i], 0.01);
		BOOST_CHECK_CLOSE(f.plog[i][1].flags, (float)f.rateFlags[i], 0.01);
	}
}

BOOST_AUTO_TEST_CASE(filterTest)
{
	//Fixture f;
	//f.v1.
}

/* Test Cases for DOF class
 *
 * Note: Compiling with -std=c++03 to match Ti's settings
 */
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE dofTest
#include <boost/test/unit_test.hpp>

#include <stdint.h>
#include <cmath>
#include "Dof.hpp"
#include "Pid.hpp"

class Fixture
{
public:
	Fixture() 
	{
		float state[NUM_DOF_STATES] = {0, 0, 0, 0};
		float setpt[NUM_DOF_STATES] = {1, 1, 1, 1}; 		
		float valueGains[NUM_PID_GAINS] = {1, 1, 1};
		float rateGains[NUM_PID_GAINS] = {1, 1, 1};
		float lpCoeff[NUM_PID_STATES - 1] = {0, 0};

		d2 = Dof(state, setpt, valueGains, rateGains, (uint8_t)DISC_DERIV_MASK,
				 (uint8_t)DERR_DT_MASK, 1, 0, (float)HUGE_VAL, -(float)HUGE_VAL, 
				 (float)HUGE_VAL, -(float)HUGE_VAL, lpCoeff, lpCoeff);
	}

	Dof d1, d2;
};

BOOST_AUTO_TEST_CASE(initTest)
{
	Fixture f;
	BOOST_CHECK_EQUAL(f.d1.getRate(), 0);
	BOOST_CHECK_EQUAL(f.d1.getRate(), 0);
	BOOST_CHECK_EQUAL(f.d2.getRate(), 0);
	BOOST_CHECK_EQUAL(f.d2.getUval(), 0);
}

BOOST_AUTO_TEST_CASE(autonCtrlTest)
{
	Fixture f;
	float setpt[NUM_DOF_STATES] = {1, 1, 1, 1}; 		
	f.d2.setSetpt(setpt, false);
	f.d2.run(false);

	BOOST_CHECK(f.d2.getRate() > 0);
	BOOST_CHECK(f.d2.getUval() > 0);
	
	float state[NUM_DOF_STATES] = {1, 1, 1, 2};
	f.d2.setState(state);
	f.d2.run(false);
	
	BOOST_CHECK(f.d2.getRate() > 0);
	BOOST_CHECK(f.d2.getUval() < 0);
}


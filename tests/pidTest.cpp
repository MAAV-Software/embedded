/* Test Cases for PID class
 *
 * Note: Compiling with -std=c++03 to match Ti's settings
 */
#define BOOST_TEST_MODULE "Pid Test"
#include <boost/test/unit_test.hpp>
#include <stdint.h>
#include <cmath>
#include "Pid.hpp"

using namespace std;


class Fixture
{
public:
	Fixture() 
	{
		float states[NUM_PID_STATES] = {0, 0, 0};
		float setpt[NUM_PID_STATES] = {1, 0, 1};
		float gains[NUM_PID_GAINS] = {1, 1, 1};
		float lpCoeff = 0.1f;

		c = Pid(states, setpt, 
				(DERR_DT_MASK | DISC_DERIV_MASK | 
				 STATE_LOWPASS_MASK | DERR_LOWPASS_MASK), 
				gains, 0, (float)HUGE_VAL, -(float)HUGE_VAL, lpCoeff, lpCoeff);
		
		p.setGains(2.0f, 2.0f, 2.0f);
		p.setSetpt(1.0f, 1.0f);
		p.setState(0, 0, 0);
	}
	
	Pid p, c; // p for plain, c for custom-constructed
};

BOOST_AUTO_TEST_CASE(initTest)
{
	Fixture f;
	BOOST_CHECK_EQUAL(f.p.getOutput(), 0);
	BOOST_CHECK_EQUAL(f.c.getOutput(), 0);
}

BOOST_AUTO_TEST_CASE(logTest)
{
	Fixture f;
	PidLog log;

	f.p.prepareLog(log);
	BOOST_CHECK_CLOSE(log.setpt, 1, 0.001);
	BOOST_CHECK_CLOSE(log.kp, 2, 0.001);
	BOOST_CHECK_CLOSE(log.ki, 2, 0.001);
	BOOST_CHECK_CLOSE(log.kd, 2, 0.001);
	BOOST_CHECK_EQUAL(log.flags, DISC_DERIV_MASK);
}

BOOST_AUTO_TEST_CASE(plainCtrlTest1)
{
	Fixture f;
	f.p.run();
	BOOST_CHECK_CLOSE(f.p.getOutput(), 3.0f, 0.001f);
}

BOOST_AUTO_TEST_CASE(plainCtrlTest2)
{
	Fixture f;
	f.p.setSetpt(-1, 1);
	f.p.run();
	BOOST_CHECK_CLOSE(f.p.getOutput(), -3.0f, 0.001f);
}


BOOST_AUTO_TEST_CASE(customCtrlTest)
{
	Fixture f;
	f.c.run();
	BOOST_CHECK_CLOSE(f.c.getOutput(), 1.6f, 0.001f);

	f.c.setState(2.5f, 0.0f, 2.5f);
	f.c.run();
	BOOST_CHECK(f.c.getOutput() < 0);

	f.c.setState(-10.0f, -1.0f, 4.0f);
	f.c.run();
	BOOST_CHECK(f.c.getOutput() > 0);
}

// End of File

/* Test Cases for PID class
 *
 * Note: Compiling with -std=c++03 to match Ti's settings
 */
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE pidTest
#include <boost/test/unit_test.hpp>
#include <stdint.h>
#include <cmath>
#include "Pid.hpp"

class Fixture
{
public:
	Fixture() 
	{
		float states[NUM_STATES] = {0, 0, 0};
		float setpt[NUM_STATES] = {1, 0, 1};
		float gains[NUM_GAINS] = {1, 1, 1};
		float lpCoeff[NUM_STATES - 1] = {0, 0};

		c = Pid(states, setpt, 
				(uint8_t)DERR_DT_MASK | (uint8_t)(DISC_DERIV_MASK), 
				gains, 0, (float)HUGE_VAL, 
				-(float)HUGE_VAL, lpCoeff);
		
		p.flags |= (uint8_t)DISC_DERIV_MASK;
		p.setGains(2.0, 2.0, 2.0);
		p.setSetpt(1.0, 1.0);
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

BOOST_AUTO_TEST_CASE(flagsTest)
{
	Fixture f;
	BOOST_CHECK((f.c.flags & (uint8_t)DERR_DT_MASK) > 0);
	BOOST_CHECK_EQUAL(f.p.flags, (uint8_t)DISC_DERIV_MASK);
	f.p.flags &= (uint8_t)(~DISC_DERIV_MASK);
	f.c.flags &= (uint8_t)(~DERR_DT_MASK);
	BOOST_CHECK_EQUAL(f.p.flags, 0);
	BOOST_CHECK_EQUAL(f.c.flags, (uint8_t)DISC_DERIV_MASK);
}

BOOST_AUTO_TEST_CASE(plainCtrlTest1)
{
	Fixture f;
	f.p.run();
	BOOST_CHECK_CLOSE(f.p.getOutput(), 3.0, 0.001);
}

BOOST_AUTO_TEST_CASE(plainCtrlTest2)
{
	Fixture f;
	f.p.setSetpt(-1, 1);
	f.p.run();
	BOOST_CHECK_CLOSE(f.p.getOutput(), -3.0, 0.001);
}


BOOST_AUTO_TEST_CASE(customCtrlTest)
{
	Fixture f;
	f.c.run();
	BOOST_CHECK_CLOSE(f.c.getOutput(), 2.5, 0.001);

	f.c.setState(2.5, 0, 2.5);
	f.c.run();
	BOOST_CHECK(f.c.getOutput() < 0);

	f.c.setState(0.5, 0.5, 4);
	f.c.run();
	BOOST_CHECK(f.c.getOutput() > 0);
}

// End of File

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
#include "FlightMode.hpp"

class Fixture
{
public:
	Fixture() 
	{
		
	}
	Dof d1, d2;
};


BOOST_AUTO_TEST_CASE(initTest)
{
	Fixture f;
	BOOST_CHECK_EQUAL(d1.getRate(), 0);
	BOOST_CHECK_EQUAL(d1.getUval(), 0);
}

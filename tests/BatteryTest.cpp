#define BOOST_TEST_MODULE "Battery Test"
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include <stdint.h>
#include "Battery.hpp"

struct Fixture
{
	Fixture()
	{
		b2 = Battery(16, 4, 1);
	}

	Battery b1, b2;
};

BOOST_AUTO_TEST_CASE(ctorTest)
{
	Fixture f;
	BOOST_CHECK_CLOSE(f.b1.getVolts(), 14.8f, 0.001);
	BOOST_CHECK_CLOSE(f.b2.getVolts(), 16, 0.001);
	BOOST_CHECK(!f.b1.isLow());
	BOOST_CHECK(!f.b2.isLow());
}
/*
BOOST_AUTO_TEST_CASE(updateTest)
{
	Fixture f;
	f.b1.update(12);
	f.b2.update(4);
	BOOST_CHECK_CLOSE(f.b1.getVolts(), 12, 0.001);
	BOOST_CHECK_CLOSE(f.b2.getVolts(), 4, 0.001);
	BOOST_CHECK(f.b1.isLow());
	BOOST_CHECK(f.b2.isLow());
}
*/

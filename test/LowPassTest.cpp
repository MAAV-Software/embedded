#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "LowPassTest"
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include "LowPass.hpp"

struct Fixture
{
	Fixture()
	{
		l2 = LowPass(0.1f, 10);
		l3 = LowPass(0.1f);
	}

	LowPass l1, l2, l3;
};

BOOST_AUTO_TEST_CASE(ctorTest)
{
	Fixture f;
	BOOST_CHECK_CLOSE(f.l1.getState(), 0, 0.0001);
	BOOST_CHECK_CLOSE(f.l2.getState(), 10, 0.0001);
	BOOST_CHECK_CLOSE(f.l3.getState(), 0, 0.0001);
}

BOOST_AUTO_TEST_CASE(runTest1)
{
	Fixture f;
	f.l1.run(10);
	f.l2.run(10);
	f.l3.run(10);

	BOOST_CHECK_CLOSE(f.l1.getState(), 0, 0.0001);
	BOOST_CHECK_CLOSE(f.l2.getState(), 10, 0.0001);
	BOOST_CHECK_CLOSE(f.l3.getState(), 1, 0.0001);
}

BOOST_AUTO_TEST_CASE(runTest2)
{
	Fixture f;
	f.l1.run(10);
	f.l2.run(10);
	f.l3.run(10);

	BOOST_CHECK_CLOSE(f.l1.getState(), 0, 0.0001);
	BOOST_CHECK_CLOSE(f.l2.getState(), 10, 0.0001);
	BOOST_CHECK_CLOSE(f.l3.getState(), 1, 0.0001);
	
	f.l1.run(20);
	f.l2.run(20);
	f.l3.run(20);
	
	BOOST_CHECK_CLOSE(f.l1.getState(), 0, 0.0001);
	BOOST_CHECK_CLOSE(f.l2.getState(), 11, 0.0001);
	BOOST_CHECK_CLOSE(f.l3.getState(), 2.9f, 0.0001);
}


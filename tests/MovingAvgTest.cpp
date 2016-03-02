#define BOOST_TEST_MODULE "MovingAvgTest"
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include "MovingAvg.hpp"

BOOST_AUTO_TEST_CASE(ctorTest)
{
	MovingAvg ma(31);
	BOOST_CHECK_CLOSE(ma.state(), 0, 0.0001);
}

BOOST_AUTO_TEST_CASE(idxStateTest)
{
	MovingAvg ma(31);
    
    for (size_t i = 0; i < 100; ++i) ma.run(1.0); 
    
	BOOST_CHECK_CLOSE(ma.state(), 1.0, 0.0001);
}

BOOST_AUTO_TEST_CASE(runTest)
{
    MovingAvg ma(31);

    BOOST_CHECK_CLOSE(ma.run(1.0), 1.0 / 31.0, 0.0001);
    BOOST_CHECK_CLOSE(ma.run(1.0), 2.0 / 31.0, 0.0001);
}

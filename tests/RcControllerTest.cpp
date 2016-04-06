#define BOOST_TEST_MODULE "RcController Test"
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include <stdint.h>

#include "MaavMath.hpp"
#include "Pair.hpp"
#include "RcController.hpp"

using Maav::Pair;
using MaavMath::map;
using namespace std;

struct Fixture
{
    Pair<uint32_t, uint32_t> profiles[2];
    uint8_t size;
    
    RcController *rc;
    
    Fixture() : size(2)
    {
        profiles[0].first  = 0;
        profiles[0].second = 100;
        
        profiles[1].first  = 100;
        profiles[1].second = 200;

        rc = new RcController(profiles, size);
    }

    ~Fixture()
    {
        delete rc;
    }
};

BOOST_AUTO_TEST_CASE(MaavMathMapTest)
{
    BOOST_CHECK_CLOSE(MaavMath::map(0.5f, 0.0f, 1.0f, 1.0f, 2.0f), 
    	1.5f, 0.0001);
    BOOST_CHECK_CLOSE(MaavMath::map(1.5f, 1.0f, 2.0f, 1.0f, 2.0f), 
    	1.5f, 0.0001);
    BOOST_CHECK_CLOSE(MaavMath::map(0.5f, 0.0f, 1.0f, 0.0f, 1.0f), 
    	0.5f, 0.0001);
    BOOST_CHECK_CLOSE(MaavMath::map(1.5f, 1.0f, 2.0f, 0.0f, 1.0f), 
    	0.5f, 0.0001);
}

BOOST_AUTO_TEST_CASE(errTest)
{
    Fixture f;

    BOOST_CHECK(f.rc->pulse(0, 3) < 0);
    BOOST_CHECK(f.rc->dutyCycle(0, 3) < 0);
}


BOOST_AUTO_TEST_CASE(minTest)
{
    Fixture f;
    
    BOOST_CHECK_CLOSE(f.rc->pulse(0, 0), 1.0f, 0.0001);
    BOOST_CHECK_CLOSE(f.rc->pulse(100, 1), 1.0f, 0.0001);
    BOOST_CHECK_CLOSE(f.rc->dutyCycle(0, 0), 0.0f, 0.0001);
    BOOST_CHECK_CLOSE(f.rc->dutyCycle(100, 1), 0.0f, 0.0001);
}

BOOST_AUTO_TEST_CASE(maxTest)
{
    Fixture f;
    
    BOOST_CHECK_CLOSE(f.rc->pulse(100, 0), 2.0f, 0.0001);
    BOOST_CHECK_CLOSE(f.rc->pulse(200, 1), 2.0f, 0.0001);
    BOOST_CHECK_CLOSE(f.rc->dutyCycle(100, 0), 1.0f, 0.0001);
    BOOST_CHECK_CLOSE(f.rc->dutyCycle(200, 1), 1.0f, 0.0001);
}

BOOST_AUTO_TEST_CASE(midTest)
{
    Fixture f;
    
    BOOST_CHECK_CLOSE(f.rc->pulse(50, 0), 1.5f, 0.0001);
    BOOST_CHECK_CLOSE(f.rc->pulse(150, 1), 1.5f, 0.0001);
    BOOST_CHECK_CLOSE(f.rc->dutyCycle(50, 0), 0.5f, 0.0001);
    BOOST_CHECK_CLOSE(f.rc->dutyCycle(150, 1), 0.5f, 0.0001);
}


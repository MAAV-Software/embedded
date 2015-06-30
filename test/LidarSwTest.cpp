#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "LidarSoftwareTest"
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include <stdint.h>
#include "Lidar.hpp"
#include "LidarDefines.hpp"

struct Fixture
{
	Fixture()
	{
		rawV[0] = (uint8_t)-12.0f;
		rawD[0] = 0x0C;
		rawD[1] = 0x0C;
		dist = float(0x0C0C) / 100.0f;
	}
	
	Lidar l;
	uint8_t rawD[LIDAR_DIST_SIZE];
	uint8_t rawV[LIDAR_VEL_SIZE];
	float dist;
};

BOOST_AUTO_TEST_CASE(ctorTest)
{
	Fixture f;
	BOOST_CHECK_EQUAL(f.l.getDist(), 0);
	BOOST_CHECK_EQUAL(f.l.getVel(), 0);
}

BOOST_AUTO_TEST_CASE(distTest)
{
	Fixture f;
	f.l.parse(f.rawD, (uint8_t)LIDAR_DIST_SIZE);
	BOOST_CHECK_CLOSE(f.l.getDist(), f.dist, 0.1);
}

BOOST_AUTO_TEST_CASE(velTest)
{
	Fixture f;
	f.l.parse(f.rawV, (uint8_t)LIDAR_VEL_SIZE);
	BOOST_CHECK_CLOSE(f.l.getVel(), (int8_t)f.rawV[0], 0.1);
}

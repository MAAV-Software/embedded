#define BOOST_TEST_MODULE "Px4SoftwareTest"
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include <cmath>
#include "Px4.hpp"
#include "Px4Defines.hpp"

struct Fixture
{
	Fixture()
	{
		b.data.frame_count = 1000;
		b.data.pixel_flow_x_sum = 20;
		b.data.pixel_flow_y_sum = 15;
		b.data.flow_comp_m_x = 10000;
		b.data.flow_comp_m_y = 15;
		b.data.qual = 100;
		b.data.gyro_x_rate = 2;
		b.data.gyro_y_rate = 1;
		b.data.gyro_z_rate = 3;
		b.data.gyro_range = 1;
		b.data.sonar_timestamp = 200;
		b.data.ground_distance = 2135;
	}

	Px4 p;
	px4FrameBuffer b;
};

BOOST_AUTO_TEST_CASE(ctorTest)
{
	Fixture f;
	BOOST_CHECK_EQUAL(f.p.getFrameCount(), 0);
	BOOST_CHECK_EQUAL(f.p.getSonarTimestamp(), 0);
	BOOST_CHECK_EQUAL(f.p.getGyroRange(), 0);
	BOOST_CHECK_CLOSE(f.p.getPixelXFlow(), 0, 0.1);
	BOOST_CHECK_CLOSE(f.p.getPixelYFlow(), 0, 0.1);
	BOOST_CHECK_CLOSE(f.p.getXFlow(), 0, 0.1);
	BOOST_CHECK_CLOSE(f.p.getYFlow(), 0, 0.1);
	BOOST_CHECK_CLOSE(f.p.getQual(), 0, 0.1);
	BOOST_CHECK_CLOSE(f.p.getGyroXRate(), 0, 0.1);
	BOOST_CHECK_CLOSE(f.p.getGyroYRate(), 0, 0.1);
	BOOST_CHECK_CLOSE(f.p.getGyroZRate(), 0, 0.1);
	BOOST_CHECK_CLOSE(f.p.getZDist(), 0, 0.1);
}

BOOST_AUTO_TEST_CASE(parseTest)
{
	Fixture f;
	
	f.p.parse(f.b.raw);
	
	BOOST_CHECK_EQUAL(f.p.getFrameCount(), 1000);
	BOOST_CHECK_EQUAL(f.p.getSonarTimestamp(), 200000);
	BOOST_CHECK_EQUAL(f.p.getGyroRange(), 1);
	BOOST_CHECK_CLOSE(f.p.getPixelXFlow(), 2, 0.1);
	BOOST_CHECK_CLOSE(f.p.getPixelYFlow(), 1.5, 0.1);
	BOOST_CHECK_CLOSE(f.p.getXFlow(), 10, 0.1);
	BOOST_CHECK_CLOSE(f.p.getYFlow(), 0.015, 0.1);
	BOOST_CHECK_CLOSE(f.p.getQual(), 100.0 / 255.0, 0.1);
	BOOST_CHECK_CLOSE(f.p.getGyroXRate(), 2, 0.1);
	BOOST_CHECK_CLOSE(f.p.getGyroYRate(), 1, 0.1);
	BOOST_CHECK_CLOSE(f.p.getGyroZRate(), 3, 0.1);
	BOOST_CHECK_CLOSE(f.p.getZDist(), 2.135, 0.1);
}

/* My First Boost Test
 *
 */
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE "Sample Test"
#include <boost/test/unit_test.hpp>

int add(int i, int j) { return i + j; }

BOOST_AUTO_TEST_CASE(sampleTest)
{
	BOOST_CHECK_EQUAL(add(2, 2), 4);
}

// End of File

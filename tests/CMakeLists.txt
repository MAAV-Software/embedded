find_package(Boost COMPONENTS unit_test_framework REQUIRED)

include_directories(SYSTEM 
                    ${PROJECT_SOURCE_DIR}/src 
					${PROJECT_SOURCE_DIR}/cmeigen
                    ${Boost_INCLUDE_DIRS} 
					${EIGEN_INCLUDE_DIR} 
                    ${LCM_INCLUDE_DIRS})

add_definitions(-D BOOST_TEST_DYN_LINK -D LINUX=1)


# Tests that don't require ctrl_sw get added here
set(TEST_NO_CTRLSW
	cmeigen_test
   )

foreach(test ${TEST_NO_CTRLSW})
	add_executable(${test} ${test}.cpp)
	target_link_libraries(${test} 
						 m 
						 cmeigen
						 stdc++
						 ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
	add_test(NAME ${test} COMMAND ${test})
endforeach()


# Tests that use both cmeigen and ctrl_sw get added here
set(TESTS
	LowPassTest
	LidarSwTest
	BatteryTest
    MovingAvgTest
    RcControllerTest
    )

configure_file(LidarCorrect.txt
    ${CMAKE_CURRENT_BINARY_DIR}/LidarCorrect.txt @ONLY)

foreach(test ${TESTS})
	add_executable(${test} ${test}.cpp)
	target_link_libraries(${test} 
						 ctrl_sw 
						 m 
						 cmeigen
						 stdc++
						 ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
	add_test(NAME ${test} COMMAND ${test})
endforeach()

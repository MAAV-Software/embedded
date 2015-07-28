include_directories(SYSTEM ${PROJECT_SOURCE_DIR}/include) 

set(TESTS
	RingBufTest
	DataLinkTest
   )

foreach(test ${TESTS})
	add_executable(${test} ${test}.cpp)
	target_link_libraries(${test} messaging)
endforeach()


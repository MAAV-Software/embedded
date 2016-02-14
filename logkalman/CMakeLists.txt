include_directories(SYSTEM 
                    	${PROJECT_SOURCE_DIR}/src 
			${PROJECT_SOURCE_DIR}/cmeigen
			${EIGEN_INCLUDE_DIR} 
                   )

add_definitions(-D LINUX=1)

add_executable(logkalman logkalman.cpp)
target_link_libraries(logkalman
	ctrl_sw
	cmeigen
	stdc++)

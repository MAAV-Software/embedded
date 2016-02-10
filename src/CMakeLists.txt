# All modules that aren't executables or hardware-dependent go here for creating
# a shared library used for linking to when doing unit tests only. The actual 
# build process for the vehicle (to build and flash flight-ready code to the
# vehicle) is currently handled by Code Composer Studio.
add_library(ctrl_sw SHARED
			kalman/KalmanFilter
			kalman/ExtendedKalmanFilter
			kalman/KalmanFunctions
			Battery
			LowPass
			Pid
			Dof
			Imu
			Px4
			Lidar
            Vehicle
            MaavMath
			messaging/DataLinkDefines
			messaging/Encoder
			messaging/Decoder
			messaging/DataLink
			messaging/lcmlite
			messaging/RingBuffer
			messaging/TransmitHandler
			messaging/MessageHandler
			messaging/emergency_t
			messaging/feedback_t
			messaging/gains_t
			messaging/raw_pose_t
			messaging/setpt_t
           )

# external include directories
include_directories(SYSTEM
	   				${PROJECT_SOURCE_DIR}/src	
	   				${PROJECT_SOURCE_DIR}/cmeigen	
					${EIGEN_INCLUDE_DIR} 
					${Boost_INCLUDE_DIRS}
                    ${LCM_INCLUDE_DIRS} 
					)
link_directories(${Boost_LIBRARY_DIRS})

add_definitions(-D LINUX=1)

# libraries to link into the ctrl_sw.so we're creating
target_link_libraries(ctrl_sw 
					  m 
					  stdc++ 
					  cmeigen 
					  ${Boost_LIBRARIES} 
					  ${LCM_LIBRARIES}
					  )

# Following is for code that is for an actual executable
set(EXES 
   )

foreach(exe ${EXES})
	add_executable(${exe} ${exe}.cpp)
	target_link_libraries(${exe} ctrl_sw)
endforeach()

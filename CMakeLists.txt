cmake_minimum_required(VERSION 2.8)
project(RexArm)

# Add path to search for modules
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${PROJECT_SOURCE_DIR}/cmake")

## Enfore C++03 compatability becuase that is what is required by the Tiva
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++03" COMPILER_SUPPORTS_CXX03)
if(COMPILER_SUPPORTS_CXX03)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++03")
else()
	message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++03
	support.  Please use a different C++ compiler.")
endif()

# -Wno-format-zero-length: permit printf("")
# -Wno-unused-parameter: permit a function to ignore an argument
# Setup CFLAGS
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror -Wvla")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pthread -Wno-format-zero-length -std=c99")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-sign-compare -Wno-unused-parameter")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-write-strings")

# Setup CXXFLAGS
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wvla -pedantic")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -Wno-format-zero-length")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-parameter")

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_AUTOMOC ON)

# Find required libs
if(CAEN)
	set(BOOST_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../boost")
endif()
find_package(Boost REQUIRED COMPONENTS program_options system)
find_package(Eigen3 REQUIRED)
#find_package(LCM REQUIRED) #Not actually needed other than for building message file

add_subdirectory(src)
add_subdirectory(cmeigen)

enable_testing()
add_subdirectory(tests)

message(STATUS "")
message(STATUS "Building a ${CMAKE_BUILD_TYPE} build")
message(STATUS "")

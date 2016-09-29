cmake_minimum_required(VERSION 2.8)
project(maavCtrl)

# Add path to search for modules
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${PROJECT_SOURCE_DIR}/cmake")

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Debug")
    	message(STATUS "Build Type Automatically set to Debug")
endif()

if(NOT TIVAWARE_INSTALL)
	message(STATUS "TIVAWARE_INSTALL not defined. Hardware Tests may not build.")
endif()


if(CMAKE_BUILD_TYPE MATCHES "Hardware")
    set(CMAKE_C_COMPILER "arm-none-eabi-gcc")
    set(CMAKE_CXX_COMPILER "arm-none-eabi-g++")
    
    # Arm/Tiva specific C flags
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=softfp")
    
    # Arm/Tiva specific CXX flags
    set(CMAKE_C_FLAGS "${CMAKE_CXX_FLAGS} -mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=softfp")

    add_definitions(-D PART_TM4C123GH6PM 
                    -D _CODE_ACCESS="\\"\\"" 
                    -D _DATA_ACCESS="\\"\\""
                    -D ARM_MATH_CM4
                    -D TARGET_IS_TM4C123_RB1 
                    -D LINKAGE
                   )
    
    message(STATUS "")
    message(STATUS "Using ${CMAKE_C_COMPILER} ${CMAKE_CXX_COMPILER}")
    message(STATUS "")
endif()

## Enfore C++03 compatability becuase that is what is required by the Tiva
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++03" COMPILER_SUPPORTS_CXX03)
if(COMPILER_SUPPORTS_CXX03)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++03")
    message(STATUS "")
    message(STATUS "Compiler Check Success: CXX03 Supported")
    message(STATUS "")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++03
    support.  Please use a different C++ compiler.")
endif()

# -Wno-format-zero-length: permit printf("")
# -Wno-unused-parameter: permit a function to ignore an argument
# Setup CFLAGS
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror -Wvla -g")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pthread -Wno-format-zero-length -std=c99")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-sign-compare -Wno-unused-parameter")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-write-strings")

# Setup CXXFLAGS
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wvla -pedantic")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -Wno-format-zero-length")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-parameter")

# src is common subdirectory to all builds
add_subdirectory(src)
add_subdirectory(doc)
add_subdirectory(lm4tools)
add_subdirectory(logkalman)
add_subdirectory(hwtests)

if (CMAKE_BUILD_TYPE MATCHES "Hardware")
    add_subdirectory(maav)
else()
    # Find required libs
    if(CAEN)
        set(BOOST_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../boost")
    endif()

    find_package(Boost REQUIRED COMPONENTS program_options system)
    find_package(Eigen3 REQUIRED)
    #find_package(LCM REQUIRED) #Not actually needed other than for building message file
    
    add_subdirectory(cmeigen)
    enable_testing()
    add_subdirectory(tests)
endif()

message(STATUS "")
message(STATUS "Building a ${CMAKE_BUILD_TYPE} build")
message(STATUS "")

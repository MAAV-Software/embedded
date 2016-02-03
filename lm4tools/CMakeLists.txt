find_package(libusb-1.0 REQUIRED)

add_subdirectory(lm4flash)
add_subdirectory(lmicdiusb)

add_custom_target(lm4tools)
add_dependencies(lm4tools lm4flash lmicdi)
	
#Have clean clean up doxygen output
add_custom_target(lm4tools_clean
				  rm -r -f ${CMAKE_CURRENT_BINARY_DIR}/lm4flash/lm4flash ${CMAKE_CURRENT_BINARY_DIR}/lmicdiusb/lmicdi
				  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
				  COMMENT "Cleaning Doxygen" VERBATIM)

add_custom_target(lm4flash 
                  COMMAND gcc -Wall -O2 -I${LIBUSB_1_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/lm4flash.c ${LIBUSB_1_LIBRARIES} -o lm4flash
		          WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
                  COMMENT "Building lm4flash" VERBATIM
                 )


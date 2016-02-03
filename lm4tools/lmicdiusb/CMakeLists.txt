add_custom_target(lmicdi 
                  COMMAND gcc -Wall -g -I${LIBUSB_1_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/lmicdi.c ${CMAKE_CURRENT_SOURCE_DIR}/socket.c ${CMAKE_CURRENT_SOURCE_DIR}/gdb.c ${LIBUSB_1_LIBRARIES} -o lmicdi
		          WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
                  COMMENT "Building lm4flash" VERBATIM
                 )

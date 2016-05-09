find_package(Doxygen)

add_custom_target(doc)

if(DOXYGEN_FOUND)
	configure_file(Doxyfile.in
		${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)

	configure_file(MicrostrainDoc
		${CMAKE_CURRENT_BINARY_DIR}/MicrostrainDoc.pdf @ONLY)

	add_custom_target(doc_doxygen
		${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
		COMMENT "Generating API documentation with Doxygen" VERBATIM)

	add_dependencies(doc doc_doxygen)

	#Have clean clean up doxygen output
	add_custom_target(doc_clean
		rm -r -f ${CMAKE_CURRENT_SOURCE_DIR}/html ${CMAKE_CURRENT_SOURCE_DIR}/latex
		rm -r -f ${CMAKE_CURRENT_BINARY_DIR}/*
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		COMMENT "Cleaning Doxygen" VERBATIM)

endif()

message("Tivaware Installation Location: "
	${TIVAWARE_INSTALL})

file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/links.txt 
"		<variable>
			<name>TIVWARE_INSTALL</name>
			<value>${TIVAWARE_INSTALL}</value>
		</variable>\n"
)

add_subdirectory(./tests)

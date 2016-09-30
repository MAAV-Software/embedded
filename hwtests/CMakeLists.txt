message("Hardware Testcase Project Generation")
message("Defined Tivaware Installation Location: "
	${TIVAWARE_INSTALL})
message("Defined Repository Root Location: "
	${REPO_LOC})

file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/vars.txt 
"		<variable>
			<name>TIVAWARE_INSTALL</name>
			<value>${TIVAWARE_INSTALL}</value>
		</variable>
		<variable>
			<name>REPO_LOC</name>
			<value>${REPO_LOC}</value>
		</variable>\n"
)

file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/links.txt
"		<link>
			<name>driverlib.lib</name>
			<type>1</type>
			<locationURI>TIVAWARE_INSTALL/driverlib/ccs/Debug/driverlib.lib</locationURI>
		</link>\n"
)

add_subdirectory(./tests)

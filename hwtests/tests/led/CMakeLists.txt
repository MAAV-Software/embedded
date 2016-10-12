file(COPY ../../project DESTINATION .)

file(READ ${CMAKE_CURRENT_SOURCE_DIR}/../../project/__.project.header CCS_DOTPROJECT_HEADER)
file(READ ${CMAKE_CURRENT_SOURCE_DIR}/../../project/__.project.midcontent CCS_DOTPROJECT_MIDCONTENT)
file(READ ${CMAKE_CURRENT_SOURCE_DIR}/../../project/__.project.linkresvarlistsep CCS_DOTPROJECT_LINKRESVARLISTSEP)
file(READ ${CMAKE_CURRENT_SOURCE_DIR}/../../project/__.project.footer CCS_DOTPROJECT_FOOTER)
file(READ ${CMAKE_CURRENT_BINARY_DIR}/../../vars.txt PROJVARS)
file(READ ${CMAKE_CURRENT_BINARY_DIR}/../../links.txt PROJLINKS)
file(STRINGS ${CMAKE_CURRENT_SOURCE_DIR}/CoverageManifest.txt COVERAGE_MANIFEST)


file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/project/.project ${CCS_DOTPROJECT_HEADER})
file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/project/.project "<name>LEDTests</name>\n")
file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/project/.project ${CCS_DOTPROJECT_MIDCONTENT})
file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/project/.project ${PROJLINKS})
foreach(TESTELEMENT ${COVERAGE_MANIFEST})
	execute_process(COMMAND echo 
"		<link>
			<name>${TESTELEMENT}</name>
			<type>1</type>
			<locationURI>REPO_LOC/${TESTELEMENT}</locationURI>
		</link>" OUTPUT_VARIABLE TESTLINK)
	file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/project/.project ${TESTLINK})
endforeach()
file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/project/.project 
"		<link>
			<name>testcase</name>
			<type>2</type>
			<locationURI>${CMAKE_CURRENT_SOURCE_DIR}</locationURI>
		</link>\n")
file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/project/.project ${CCS_DOTPROJECT_LINKRESVARLISTSEP})
file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/project/.project ${PROJVARS})
file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/project/.project ${CCS_DOTPROJECT_FOOTER})

file(COPY ../../project DESTINATION .)

file(READ ../../project/__.project.header CCS_DOTPROJECT_HEADER)
file(READ ../../project/__.project.midcontent CCS_DOTPROJECT_MIDCONTENT)
file(READ ../../project/__.project.linkresvarlistsep CCS_DOTPROJECT_LINKRESVARLISTSEP)
file(READ ../../project/__.project.footer CCS_DOTPROJECT_FOOTER)
file(READ ${CMAKE_CURRENT_BINARY_DIR}/../../links.txt LINKRES)

file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/project/.project ${CCS_DOTPROJECT_HEADER})
file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/project/.project "<name>RingbufferTests</name>\n")
file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/project/.project ${CCS_DOTPROJECT_MIDCONTENT})
file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/project/.project ${CCS_DOTPROJECT_LINKRESVARLISTSEP})
file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/project/.project ${LINKRES})
file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/project/.project ${CCS_DOTPROJECT_FOOTER})

message(asdf ${CCS_DOTPROJECT_HEADER})
message("ASDFASDFASDFASDFASDF")

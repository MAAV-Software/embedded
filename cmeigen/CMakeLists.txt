# all modules that aren't executables go here
add_library(cmeigen SHARED
			cmeigen
           )

# external include directories
include_directories(SYSTEM 
					${EIGEN_INCLUDE_DIR} 
					${PROJECT_SOURCE_DIR}/cmeigen
					)

# libraries to link into the cmeigen.so we're creating
target_link_libraries(cmeigen m stdc++)

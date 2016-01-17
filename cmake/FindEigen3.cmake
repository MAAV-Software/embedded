find_path(EIGEN_INCLUDE_DIR Eigen
	PATH_SUFFIXES eigen3
	PATHS
	/usr/include
	/usr/local/include
	${CMAKE_CURRENT_SOURCE_DIR}/../)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Eigen DEFAULT_MSG EIGEN_INCLUDE_DIR)

mark_as_advanced(EIGEN_INCLUDE_DIR)

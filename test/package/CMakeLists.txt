cmake_minimum_required(VERSION 3.22)
project(v3_block_grid_package_consumer LANGUAGES C CXX)

find_package(box3d CONFIG REQUIRED)

get_target_property(box3d_type box3d::box3d TYPE)
if(NOT box3d_type STREQUAL "SHARED_LIBRARY")
	message(FATAL_ERROR "Package test requires a shared library, got ${box3d_type}")
endif()

add_executable(v3_block_grid_package_c block_grid_package_c.c)
set_target_properties(v3_block_grid_package_c PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED YES C_EXTENSIONS NO)
target_link_libraries(v3_block_grid_package_c PRIVATE box3d::box3d)

add_executable(v3_block_grid_package_cpp block_grid_package_cpp.cpp)
set_target_properties(v3_block_grid_package_cpp PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)
target_link_libraries(v3_block_grid_package_cpp PRIVATE box3d::box3d)

if(WIN32)
	# The installed DLL must accompany the external consumer executables.
	add_custom_command(TARGET v3_block_grid_package_c POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_if_different
			$<TARGET_FILE:box3d::box3d> $<TARGET_FILE_DIR:v3_block_grid_package_c>
		VERBATIM
	)
endif()

add_custom_target(v3_block_grid_package_check ALL
	COMMAND $<TARGET_FILE:v3_block_grid_package_c>
	COMMAND $<TARGET_FILE:v3_block_grid_package_cpp>
	DEPENDS v3_block_grid_package_c v3_block_grid_package_cpp
	VERBATIM
)

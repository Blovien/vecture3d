cmake_minimum_required(VERSION 3.22)

project(v3_java_test_fixtures LANGUAGES C)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

add_library(vecture3d_mismatch SHARED v3_mismatch.c)
add_library(vecture3d_null_world SHARED v3_null_world.c)
target_include_directories(vecture3d_null_world PRIVATE "${CMAKE_CURRENT_LIST_DIR}/../../main/public")

foreach(target vecture3d_mismatch vecture3d_null_world)
	set_target_properties(${target} PROPERTIES
		C_EXTENSIONS NO
		C_STANDARD 17
		C_STANDARD_REQUIRED YES
		WINDOWS_EXPORT_ALL_SYMBOLS YES
	)
	if(MSVC)
		target_compile_options(${target} PRIVATE /W4 /WX)
	else()
		target_compile_options(${target} PRIVATE -Wall -Wextra -Werror)
	endif()
endforeach()

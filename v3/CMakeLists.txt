set_property(TARGET box3d PROPERTY POSITION_INDEPENDENT_CODE ON)

add_library(vecture3d_native SHARED
	src/v3_abi.c
	src/v3_world.c
)
target_compile_features(vecture3d_native PRIVATE c_std_17)
target_include_directories(vecture3d_native
	PUBLIC "${PROJECT_SOURCE_DIR}/ffi-java/src/main/public"
	PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
target_link_libraries(vecture3d_native PRIVATE box3d::box3d)
set_target_properties(vecture3d_native PROPERTIES
	C_EXTENSIONS OFF
	C_VISIBILITY_PRESET hidden
	OUTPUT_NAME vecture3d
	VISIBILITY_INLINES_HIDDEN YES
)

add_executable(v3_native_tests test/test_world.c)
target_compile_features(v3_native_tests PRIVATE c_std_17)
target_include_directories(v3_native_tests PRIVATE "${PROJECT_SOURCE_DIR}/ffi-java/src/main/public")
target_link_libraries(v3_native_tests PRIVATE vecture3d_native)

if(MSVC)
	target_compile_options(vecture3d_native PRIVATE /W4 /WX /fp:strict)
	target_compile_options(v3_native_tests PRIVATE /W4 /WX /fp:strict)
else()
	target_compile_options(vecture3d_native PRIVATE -Wall -Wextra -Wpedantic -Werror -fno-fast-math)
	target_compile_options(v3_native_tests PRIVATE -Wall -Wextra -Wpedantic -Werror -fno-fast-math)
endif()

add_test(NAME v3_world COMMAND v3_native_tests)

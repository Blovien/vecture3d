set_property(TARGET box3d PROPERTY POSITION_INDEPENDENT_CODE ON)

add_library(vecture3d_native SHARED
	src/v3_abi.c
	src/v3_geometry.c
	src/v3_joint.c
	src/v3_step.c
	src/v3_world.c
)
target_include_directories(vecture3d_native
	PUBLIC "${PROJECT_SOURCE_DIR}/ffi-java/src/main/public"
	PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
target_link_libraries(vecture3d_native PRIVATE box3d::box3d)
set_target_properties(vecture3d_native PROPERTIES
	C_EXTENSIONS NO
	C_STANDARD 17
	C_STANDARD_REQUIRED YES
	C_VISIBILITY_PRESET hidden
	OUTPUT_NAME vecture3d
	VISIBILITY_INLINES_HIDDEN YES
)

add_executable(v3_native_tests test/test_world.c)
target_include_directories(v3_native_tests PRIVATE "${PROJECT_SOURCE_DIR}/ffi-java/src/main/public")
target_link_libraries(v3_native_tests PRIVATE vecture3d_native)
set_target_properties(v3_native_tests PROPERTIES
	C_EXTENSIONS NO
	C_STANDARD 17
	C_STANDARD_REQUIRED YES
)

add_executable(v3_geometry_tests
	test/test_geometry.c
	src/v3_abi.c
	src/v3_geometry.c
	src/v3_joint.c
	src/v3_step.c
	src/v3_world.c
)
target_include_directories(v3_geometry_tests PRIVATE
	"${PROJECT_SOURCE_DIR}/ffi-java/src/main/public"
	"${CMAKE_CURRENT_SOURCE_DIR}/src"
)
target_link_libraries(v3_geometry_tests PRIVATE box3d::box3d)
set_target_properties(v3_geometry_tests PROPERTIES
	C_EXTENSIONS NO
	C_STANDARD 17
	C_STANDARD_REQUIRED YES
	C_VISIBILITY_PRESET hidden
)

add_executable(v3_joint_tests
	test/test_joint.c
	src/v3_abi.c
	src/v3_geometry.c
	src/v3_joint.c
	src/v3_step.c
	src/v3_world.c
)
target_include_directories(v3_joint_tests PRIVATE
	"${PROJECT_SOURCE_DIR}/ffi-java/src/main/public"
	"${CMAKE_CURRENT_SOURCE_DIR}/src"
)
target_link_libraries(v3_joint_tests PRIVATE box3d::box3d)
set_target_properties(v3_joint_tests PROPERTIES
	C_EXTENSIONS NO
	C_STANDARD 17
	C_STANDARD_REQUIRED YES
	C_VISIBILITY_PRESET hidden
)

add_executable(v3_step_tests
	test/test_step.c
	src/v3_abi.c
	src/v3_geometry.c
	src/v3_joint.c
	src/v3_step.c
	src/v3_world.c
)
target_include_directories(v3_step_tests PRIVATE
	"${PROJECT_SOURCE_DIR}/ffi-java/src/main/public"
	"${CMAKE_CURRENT_SOURCE_DIR}/src"
)
target_link_libraries(v3_step_tests PRIVATE box3d::box3d)
set_target_properties(v3_step_tests PROPERTIES
	C_EXTENSIONS NO
	C_STANDARD 17
	C_STANDARD_REQUIRED YES
	C_VISIBILITY_PRESET hidden
)

add_executable(v3_world_internal_tests
	test/test_world_internal.c
	src/v3_world.c
)
target_compile_definitions(v3_world_internal_tests PRIVATE V3_TESTING)
target_include_directories(v3_world_internal_tests PRIVATE
	"${PROJECT_SOURCE_DIR}/ffi-java/src/main/public"
	"${CMAKE_CURRENT_SOURCE_DIR}/src"
)
target_link_libraries(v3_world_internal_tests PRIVATE box3d::box3d)
set_target_properties(v3_world_internal_tests PROPERTIES
	C_EXTENSIONS NO
	C_STANDARD 17
	C_STANDARD_REQUIRED YES
	C_VISIBILITY_PRESET hidden
)

if(MSVC)
	target_compile_options(vecture3d_native PRIVATE /W4 /WX /fp:strict)
	target_compile_options(v3_native_tests PRIVATE /W4 /WX /fp:strict)
	target_compile_options(v3_geometry_tests PRIVATE /W4 /WX /fp:strict)
	target_compile_options(v3_joint_tests PRIVATE /W4 /WX /fp:strict)
	target_compile_options(v3_step_tests PRIVATE /W4 /WX /fp:strict)
	target_compile_options(v3_world_internal_tests PRIVATE /W4 /WX /fp:strict)
else()
	target_compile_options(vecture3d_native PRIVATE -Wall -Wextra -Wpedantic -Werror -fno-fast-math)
	target_compile_options(v3_native_tests PRIVATE -Wall -Wextra -Wpedantic -Werror -fno-fast-math)
	target_compile_options(v3_geometry_tests PRIVATE -Wall -Wextra -Wpedantic -Werror -fno-fast-math)
	target_compile_options(v3_joint_tests PRIVATE -Wall -Wextra -Wpedantic -Werror -fno-fast-math)
	target_compile_options(v3_step_tests PRIVATE -Wall -Wextra -Wpedantic -Werror -fno-fast-math)
	target_compile_options(v3_world_internal_tests PRIVATE -Wall -Wextra -Wpedantic -Werror -fno-fast-math)
endif()

add_test(NAME v3_world COMMAND v3_native_tests)
add_test(NAME v3_geometry COMMAND v3_geometry_tests)
add_test(NAME v3_joint COMMAND v3_joint_tests)
add_test(NAME v3_step COMMAND v3_step_tests)
add_test(NAME v3_world_internal COMMAND v3_world_internal_tests)

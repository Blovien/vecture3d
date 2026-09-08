# Box3D unit test app

set(BOX3D_TEST_FILES
    main.c
	test_allocator.c
    test_bitset.c
	test_body.c
	test_body_query.c
	test_collision.c
	test_compound.c
	test_container.c
    test_determinism.c
    test_distance.c
    test_hash.c
    test_height_field.c
    test_hull.c
    test_id.c
    test_joint.c
    test_large_world.c
    test_macros.h
    test_manifold.c
    test_math.c
    test_mesh.c
	test_mover.c
    test_name_cache.c
    test_recording.c
    test_sat.c
    test_shape.c
    test_table.c
    test_v3_block_grid_contacts.c
    test_v3_block_grid_world.c
    test_v3_block_grid_landing.c
    test_v3_block_grid_replace.c
    test_v3_block_grid_sweep.c
    test_v3_block_grid_motion.c
    test_v3_block_grid_events.c
	test_v3_block_grid_casts.c
    test_v3_block_grid_entry.c
    test_v3_world_limits.c
    test_world.c
)

add_executable(box3d_test ${BOX3D_TEST_FILES})

set_target_properties(box3d_test PROPERTIES
    C_STANDARD 17
    C_STANDARD_REQUIRED YES
    C_EXTENSIONS NO
)

# Differentiate between Box3D tests and ours
add_custom_command(TARGET box3d_test POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:box3d_test>" "$<TARGET_FILE_DIR:box3d_test>/test"
)

# Special access to Box3D internals for testing
target_include_directories(box3d_test PRIVATE ${CMAKE_SOURCE_DIR}/src)

if(BOX3D_VALIDATE)
	target_compile_definitions(box3d_test PRIVATE BOX3D_VALIDATE)
endif()

if(BOX3D_DISABLE_SIMD)
	target_compile_definitions(box3d_test PRIVATE BOX3D_DISABLE_SIMD)
endif()

target_link_libraries(box3d_test PRIVATE box3d shared)

add_test(NAME box3d COMMAND box3d_test)

add_executable(v3_block_grid_cook_contract_c test_v3_block_grid_cook_contract.c)
set_target_properties(v3_block_grid_cook_contract_c PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED YES C_EXTENSIONS NO)
target_include_directories(v3_block_grid_cook_contract_c PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(v3_block_grid_cook_contract_c PRIVATE box3d)
add_test(NAME v3_block_grid_cook_contract_c COMMAND v3_block_grid_cook_contract_c)

find_package(Threads REQUIRED)
add_executable(v3_block_grid_cook_contract_cpp test_v3_block_grid_cook_contract.cpp)
set_target_properties(v3_block_grid_cook_contract_cpp PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)
target_link_libraries(v3_block_grid_cook_contract_cpp PRIVATE box3d Threads::Threads)
add_test(NAME v3_block_grid_cook_contract_cpp COMMAND v3_block_grid_cook_contract_cpp)

add_executable(v3_block_grid_attach_contract_c test_v3_block_grid_attach_contract.c)
set_target_properties(v3_block_grid_attach_contract_c PROPERTIES C_STANDARD 17 C_STANDARD_REQUIRED YES C_EXTENSIONS NO)
target_link_libraries(v3_block_grid_attach_contract_c PRIVATE box3d)
add_test(NAME v3_block_grid_attach_contract_c COMMAND v3_block_grid_attach_contract_c)

configure_file(test_v3_block_grid_package.cmake.in v3_block_grid_package_test.cmake @ONLY)
add_test(NAME v3_block_grid_installed_package
	COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/v3_block_grid_package_test.cmake)
set_tests_properties(v3_block_grid_installed_package PROPERTIES LABELS "blockgrid;package" RUN_SERIAL TRUE)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "" FILES ${BOX3D_TEST_FILES})

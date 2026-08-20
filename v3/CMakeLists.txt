add_executable(v3_abi_layout_test test/test_abi_layout.c)
target_compile_features(v3_abi_layout_test PRIVATE c_std_17)
target_include_directories(v3_abi_layout_test PRIVATE "${PROJECT_SOURCE_DIR}/ffi-java/src/main/public")

add_test(NAME v3_abi_layout COMMAND v3_abi_layout_test)

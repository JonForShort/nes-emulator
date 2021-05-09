cmake_minimum_required (VERSION 3.14)

project (flutter-native-jones)

set(native_jones_root_dir ${CMAKE_CURRENT_SOURCE_DIR}/../../../../..)

set(native_jones_bin_dir ${CMAKE_CURRENT_SOURCE_DIR}/build)

add_subdirectory (${native_jones_root_dir} ${native_jones_bin_dir})

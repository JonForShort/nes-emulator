cmake_minimum_required (VERSION 3.7)

# boost

set (boost-include ${DIR_ROOT_OUT}/boost/include)
set (boost-lib     ${DIR_ROOT_OUT}/boost/lib)

# zlib

set (zlib-include ${DIR_ROOT_OUT}/zlib/include)
set (zlib-lib     ${DIR_ROOT_OUT}/zlib/lib)

# nes_test

set (sources traces.cc)

add_executable (test_suite_traces ${sources})

target_link_libraries (test_suite_traces log)
target_link_libraries (test_suite_traces jones-lib)
target_link_libraries (test_suite_traces ${boost-lib}/libboost_iostreams.a)
target_link_libraries (test_suite_traces ${boost-lib}/libboost_filesystem.a)
target_link_libraries (test_suite_traces ${boost-lib}/libboost_unit_test_framework.a)
target_link_libraries (test_suite_traces ${zlib-lib}/libz.a)

target_include_directories (test_suite_traces PRIVATE ${zlib-include})
target_include_directories (test_suite_traces PRIVATE ${boost-include})

add_test(NAME test_suite_traces COMMAND test_suite_traces ${CMAKE_CURRENT_SOURCE_DIR})

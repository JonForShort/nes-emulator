cmake_minimum_required (VERSION 3.7)

set (sources memory.cc)

add_library (memory STATIC ${sources})

target_include_directories (memory PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

include (${CMAKE_CURRENT_SOURCE_DIR}/test.cmake)

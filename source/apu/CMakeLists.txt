cmake_minimum_required (VERSION 3.7)

set (sources apu.cc)

add_library (apu STATIC ${sources})

target_include_directories (apu PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

include (${CMAKE_CURRENT_SOURCE_DIR}/test.cmake)

cmake_minimum_required (VERSION 3.7)

set (sources ppu.cc)

add_library (ppu STATIC ${sources})

target_include_directories (ppu PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

include (${CMAKE_CURRENT_SOURCE_DIR}/test.cmake)

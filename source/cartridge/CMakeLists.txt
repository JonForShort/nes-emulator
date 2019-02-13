cmake_minimum_required (VERSION 3.7)

set (sources cartridge.cc)

add_library (cartridge STATIC ${sources})

target_include_directories (cartridge PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

include (${CMAKE_CURRENT_SOURCE_DIR}/test.cmake)

cmake_minimum_required (VERSION 3.7)

set (sources controller.cc)

add_library (controller STATIC ${sources})

target_include_directories (controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

include (${CMAKE_CURRENT_SOURCE_DIR}/test.cmake)

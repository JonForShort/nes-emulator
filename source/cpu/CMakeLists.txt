cmake_minimum_required (VERSION 3.7)

set (sources
  cpu.cc
  decode.cc
  disassemble.cc
)

add_library (cpu STATIC ${sources})

target_include_directories (cpu PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

include (${CMAKE_CURRENT_SOURCE_DIR}/test.cmake)

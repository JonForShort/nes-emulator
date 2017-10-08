cmake_minimum_required(VERSION 3.7)

include_directories (include)

set (sources main.cc nes_rom.cc)

add_executable (jones ${sources})

include (${CMAKE_CURRENT_SOURCE_DIR}/sdl_module.cmake)

cmake_minimum_required (VERSION 3.7)

include_directories (include)

set (sources main.cc nes_rom.cc screen.cc)

add_executable (jones ${sources})

target_link_libraries (jones cpu)

include (${CMAKE_CURRENT_SOURCE_DIR}/sdl_module.cmake)

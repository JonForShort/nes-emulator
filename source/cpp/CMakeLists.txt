cmake_minimum_required(VERSION 3.7)

project (jones)

include_directories (include)

set (sources main.cc nes_rom.cc)

add_executable (jones ${sources})

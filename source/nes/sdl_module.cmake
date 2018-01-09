cmake_minimum_required(VERSION 3.7)

include(FindPkgConfig)

pkg_search_module(SDL2 REQUIRED sdl2)

include_directories(${SDL2_INCLUDE_DIRS})

target_link_libraries(jones ${SDL2_LIBRARIES})

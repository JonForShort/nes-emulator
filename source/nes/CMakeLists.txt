cmake_minimum_required (VERSION 3.7)

# sdl
set (sdl-include   ${DIR_ROOT_OUT}/sdl/include)
set (sdl-lib       ${DIR_ROOT_OUT}/sdl/lib)

# sndio
set (sndio-include ${DIR_ROOT_OUT}/sndio/include)
set (sndio-lib     ${DIR_ROOT_OUT}/sndio/lib)

set (sources main.cc nes_rom.cc screen.cc)

add_executable (jones ${sources})

add_dependencies (jones sndio)
add_dependencies (jones sdl)

target_link_libraries (jones cpu)
target_link_libraries (jones ${sdl-lib}/libSDL2.so)
target_link_libraries (jones ${sndio-lib}/libsndio.so)
target_link_libraries (jones ${CMAKE_DL_LIBS})
target_link_libraries (jones pthread)

target_include_directories (jones PRIVATE ${sdl-include})
target_include_directories (jones PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

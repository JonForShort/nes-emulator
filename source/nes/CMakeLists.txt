cmake_minimum_required (VERSION 3.7)

# boost

set (boost-include  ${DIR_ROOT_OUT}/boost/include)
set (boost-lib      ${DIR_ROOT_OUT}/boost/lib)

# sdl

set (sdl-include   ${DIR_ROOT_OUT}/sdl/include)
set (sdl-lib       ${DIR_ROOT_OUT}/sdl/lib)

# sndio

set (sndio-include ${DIR_ROOT_OUT}/sndio/include)
set (sndio-lib     ${DIR_ROOT_OUT}/sndio/lib)

# jones library

set (jones-lib-source nes.cc nes_rom.cc screen.cc)

add_library (jones-lib SHARED ${jones-lib-source})

add_dependencies (jones-lib boost)
add_dependencies (jones-lib sndio)
add_dependencies (jones-lib sdl)

target_link_libraries (jones-lib memory)
target_link_libraries (jones-lib apu)
target_link_libraries (jones-lib cpu)
target_link_libraries (jones-lib ppu)
target_link_libraries (jones-lib ${sdl-lib}/libSDL2.so)
target_link_libraries (jones-lib ${sndio-lib}/libsndio.so)
target_link_libraries (jones-lib ${CMAKE_DL_LIBS})
target_link_libraries (jones-lib pthread)

target_include_directories (jones-lib PRIVATE ${boost-include})
target_include_directories (jones-lib PRIVATE ${sdl-include})
target_include_directories (jones-lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# jones executable

set (jones-source main.cc)

add_executable (jones ${jones-source})

target_link_libraries (jones jones-lib)

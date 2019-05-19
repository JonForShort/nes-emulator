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

set (jones-source main.cc)

add_executable (jones ${jones-source})

target_link_libraries (jones jones-lib)
target_include_directories (jones PRIVATE jones-lib)

target_link_libraries (jones ${boost-lib}/libboost_filesystem.a)
target_link_libraries (jones ${boost-lib}/libboost_program_options.a)
target_link_libraries (jones ${sdl-lib}/libSDL2.so)
target_link_libraries (jones ${sndio-lib}/libsndio.so)
target_link_libraries (jones ${CMAKE_DL_LIBS})
target_link_libraries (jones pthread)

target_include_directories (jones PRIVATE ${boost-include})
target_include_directories (jones PRIVATE ${sdl-include})


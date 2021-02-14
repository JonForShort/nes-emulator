cmake_minimum_required (VERSION 3.14)

# boost

set (boost-include  ${DIR_ROOT_OUT}/boost/include)
set (boost-lib      ${DIR_ROOT_OUT}/boost/lib)

# jones library

set (source nes.cc)

add_library (jones-lib SHARED ${source})

add_dependencies (jones-lib boost)

target_link_libraries (jones-lib memory)
target_link_libraries (jones-lib apu)
target_link_libraries (jones-lib cpu)
target_link_libraries (jones-lib cartridge)
target_link_libraries (jones-lib controller)
target_link_libraries (jones-lib ppu)
target_link_libraries (jones-lib screen)
target_link_libraries (jones-lib ${boost-lib}/libboost_filesystem.a)

target_include_directories (jones-lib PRIVATE ${boost-include})
target_include_directories (jones-lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

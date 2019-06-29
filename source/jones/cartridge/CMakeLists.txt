cmake_minimum_required (VERSION 3.7)

# boost

set (boost-include  ${DIR_ROOT_OUT}/boost/include)
set (boost-lib      ${DIR_ROOT_OUT}/boost/lib)

# cartridge

set (sources
        cartridge.cc
        cartridge_header.cc
        mapper/mapper.cc
        mapper/mapper_unsupported.cc
        mapper/mapper_nrom.cc
        mapper/mapper_mmc1.cc)

add_library (cartridge STATIC ${sources})

target_link_libraries (cartridge configuration)
target_link_libraries (cartridge memory)

target_include_directories (cartridge PRIVATE ${boost-include})
target_include_directories (cartridge PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

include (${CMAKE_CURRENT_SOURCE_DIR}/test.cmake)

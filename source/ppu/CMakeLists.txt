cmake_minimum_required (VERSION 3.7)

# boost

set (boost-include  ${DIR_ROOT_OUT}/boost/include)
set (boost-lib      ${DIR_ROOT_OUT}/boost/lib)

# ppu

set (sources ppu.cc control_register.cc mask_register.cc status_register.cc)

add_library (ppu STATIC ${sources})

target_link_libraries (ppu memory)
target_link_libraries (ppu screen)

target_include_directories (ppu PRIVATE ${boost-include})
target_include_directories (ppu PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

include (${CMAKE_CURRENT_SOURCE_DIR}/test.cmake)

cmake_minimum_required (VERSION 3.7)

# boost

set (boost-include  ${DIR_ROOT_OUT}/boost/include)
set (boost-lib      ${DIR_ROOT_OUT}/boost/lib)

# apu

set (sources apu.cc)

add_library (apu STATIC ${sources})

target_link_libraries (apu memory)

target_include_directories (apu PRIVATE ${boost-include})
target_include_directories (apu PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

include (${CMAKE_CURRENT_SOURCE_DIR}/test.cmake)

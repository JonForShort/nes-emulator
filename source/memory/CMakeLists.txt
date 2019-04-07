cmake_minimum_required (VERSION 3.7)

# boost

set (boost-include  ${DIR_ROOT_OUT}/boost/include)
set (boost-lib      ${DIR_ROOT_OUT}/boost/lib)

# memory

set (sources memory.cc)

add_library (memory STATIC ${sources})

target_include_directories (memory PRIVATE ${boost-include})
target_include_directories (memory PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

include (${CMAKE_CURRENT_SOURCE_DIR}/test.cmake)

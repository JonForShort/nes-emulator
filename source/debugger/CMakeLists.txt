cmake_minimum_required (VERSION 3.7)

# boost

set (boost-include  ${DIR_ROOT_OUT}/boost/include)
set (boost-lib      ${DIR_ROOT_OUT}/boost/lib)

# debugger

set (sources debugger.cc)

add_library (debugger STATIC ${sources})

target_link_libraries (debugger memory)

target_include_directories (debugger PRIVATE ${boost-include})
target_include_directories (debugger PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

include (${CMAKE_CURRENT_SOURCE_DIR}/test.cmake)

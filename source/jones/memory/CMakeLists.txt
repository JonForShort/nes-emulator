cmake_minimum_required(VERSION 3.14)

# boost

set(boost-include ${DIR_ROOT_OUT}/boost/include)
set(boost-lib ${DIR_ROOT_OUT}/boost/lib)

# memory

set(sources memory.cc)

add_library(memory STATIC ${sources})

add_dependencies(memory boost)

target_link_libraries(memory log)

target_include_directories(memory PRIVATE ${boost-include})
target_include_directories(memory PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

include(${CMAKE_CURRENT_SOURCE_DIR}/test.cmake)

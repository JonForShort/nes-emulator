cmake_minimum_required (VERSION 3.7)

# boost

set (boost-include  ${DIR_ROOT_OUT}/boost/include)
set (boost-lib      ${DIR_ROOT_OUT}/boost/lib)

set (sources controller.cc)

add_library (controller STATIC ${sources})

target_link_libraries (controller log)
target_link_libraries (controller memory)

target_include_directories (controller PRIVATE ${boost-include})
target_include_directories (controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

include (${CMAKE_CURRENT_SOURCE_DIR}/test.cmake)

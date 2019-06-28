cmake_minimum_required (VERSION 3.7)

# boost

set (boost-include  ${DIR_ROOT_OUT}/boost/include)
set (boost-lib      ${DIR_ROOT_OUT}/boost/lib)

# configuration

set (sources configuration.cc)

add_library (configuration STATIC ${sources})

target_include_directories (configuration PRIVATE ${boost-include})
target_include_directories (configuration PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

include (${CMAKE_CURRENT_SOURCE_DIR}/test.cmake)

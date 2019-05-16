cmake_minimum_required (VERSION 3.7)

# boost

set (boost-include  ${DIR_ROOT_OUT}/boost/include)
set (boost-lib      ${DIR_ROOT_OUT}/boost/lib)

# screen

set (sources screen.cc)

add_library (screen STATIC ${sources})

target_include_directories (screen PRIVATE ${boost-include})
target_include_directories (screen PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

include (${CMAKE_CURRENT_SOURCE_DIR}/test.cmake)

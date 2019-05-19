cmake_minimum_required (VERSION 3.7)

project (jones)

enable_testing()

set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_VERBOSE_MAKEFILE ON)
set (CMAKE_POSITION_INDEPENDENT_CODE ON)
set (CMAKE_EXPORT_COMPILE_COMMANDS ON)

include (${CMAKE_CURRENT_SOURCE_DIR}/cmake/CustomWarnings.cmake)
include (${CMAKE_CURRENT_SOURCE_DIR}/cmake/CodeCoverage.cmake)
include (${CMAKE_CURRENT_SOURCE_DIR}/cmake/AddressSanitizer.cmake)
include (${CMAKE_CURRENT_SOURCE_DIR}/cmake/CustomFlags.cmake)

set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/out/${CMAKE_PROJECT_NAME}/lib)
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/out/${CMAKE_PROJECT_NAME}/lib)
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/out/${CMAKE_PROJECT_NAME}/bin)

set (DIR_ROOT_OUT      ${CMAKE_CURRENT_SOURCE_DIR}/out)
set (DIR_ROOT_TEST     ${CMAKE_CURRENT_SOURCE_DIR}/test)
set (DIR_ROOT_SOURCE   ${CMAKE_CURRENT_SOURCE_DIR}/source)
set (DIR_ROOT_EXTERNAL ${CMAKE_CURRENT_SOURCE_DIR}/external)

add_subdirectory (external)
add_subdirectory (source/jones)
add_subdirectory (source/tools)
add_subdirectory (source/frontends)
add_subdirectory (test/cpu)

cmake_minimum_required (VERSION 3.14)

project (jones)

enable_testing()

set (CMAKE_CXX_STANDARD 20)
set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_VERBOSE_MAKEFILE ON)
set (CMAKE_POSITION_INDEPENDENT_CODE ON)
set (CMAKE_EXPORT_COMPILE_COMMANDS ON)

set_property (GLOBAL PROPERTY GLOBAL_DEPENDS_DEBUG_MODE 0)

include (${CMAKE_CURRENT_SOURCE_DIR}/cmake/CustomWarnings.cmake)
include (${CMAKE_CURRENT_SOURCE_DIR}/cmake/CodeCoverage.cmake)
include (${CMAKE_CURRENT_SOURCE_DIR}/cmake/AddressSanitizer.cmake)
include (${CMAKE_CURRENT_SOURCE_DIR}/cmake/CustomFlags.cmake)
include (${CMAKE_CURRENT_SOURCE_DIR}/cmake/platform/Default.cmake)
include (${CMAKE_CURRENT_SOURCE_DIR}/cmake/platform/Android.cmake)

set (DIR_ROOT_OUT      ${JONES_DIR_ROOT_OUT})
set (DIR_ROOT_TEST     ${CMAKE_CURRENT_SOURCE_DIR}/test)
set (DIR_ROOT_SOURCE   ${CMAKE_CURRENT_SOURCE_DIR}/source)
set (DIR_ROOT_EXTERNAL ${CMAKE_CURRENT_SOURCE_DIR}/external)

add_subdirectory (external)
add_subdirectory (source/jones)
add_subdirectory (source/tools)
add_subdirectory (source/frontends)
add_subdirectory (test/traces)
add_subdirectory (test/cpu)

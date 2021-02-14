cmake_minimum_required (VERSION 3.14)

# util library

set (source util.cc)

add_library (util SHARED ${source})

target_include_directories (util PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

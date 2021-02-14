cmake_minimum_required (VERSION 3.14)

add_subdirectory (source)

add_custom_target (cppcoro-headers COMMAND ${CMAKE_COMMAND}
  -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/source/include ${DIR_PROJECT_OUT}/include)

add_dependencies (cppcoro cppcoro-headers)

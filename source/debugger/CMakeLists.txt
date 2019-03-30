cmake_minimum_required (VERSION 3.7)

set (source-files
  debugger.cc
)

add_library (jones-debugger SHARED ${source-files})

target_link_libraries (jones-debugger cpu)
target_link_libraries (jones-debugger log)

target_include_directories (jones-debugger PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_custom_target (jones-debugger-copy-headers COMMAND ${CMAKE_COMMAND}
        -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/include ${DIR_ROOT_OUT}/jones/include/jones-debugger)

add_dependencies(jones-debugger jones-debugger-copy-headers)

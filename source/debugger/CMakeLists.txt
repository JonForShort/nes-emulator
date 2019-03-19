cmake_minimum_required (VERSION 3.7)

# boost
set (boost-include   ${DIR_ROOT_OUT}/boost/include)
set (boost-lib       ${DIR_ROOT_OUT}/boost/lib)

# ncurses
set (ncurses-include ${DIR_ROOT_OUT}/ncurses/include)
set (ncurses-lib     ${DIR_ROOT_OUT}/ncurses/lib)

set (source-files
  debugger.cc
  interface.cc
  windows/command/command_buffer.cc
  windows/command/command_prompt.cc
  windows/command/command_window.cc
  windows/content/content_window.cc
)

add_executable (jones-debugger ${source-files})

add_dependencies (jones-debugger boost)
add_dependencies (jones-debugger ncurses)

target_link_libraries (jones-debugger cpu)
target_link_libraries (jones-debugger log)
target_link_libraries (jones-debugger ${boost-lib}/libboost_program_options.a)
target_link_libraries (jones-debugger ${boost-lib}/libboost_filesystem.a)
target_link_libraries (jones-debugger ${ncurses-lib}/libncurses.a)
target_link_libraries (jones-debugger pthread)

target_include_directories (jones-debugger PRIVATE ${boost-include})
target_include_directories (jones-debugger PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories (jones-debugger PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/windows/include)

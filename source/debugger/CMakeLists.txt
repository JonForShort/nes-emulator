cmake_minimum_required (VERSION 3.7)

# boost
set (boost-include   ${DIR_ROOT_OUT}/boost/include)
set (boost-lib       ${DIR_ROOT_OUT}/boost/lib)

# ncurses
set (ncurses-include ${DIR_ROOT_OUT}/ncurses/include)
set (ncurses-lib     ${DIR_ROOT_OUT}/ncurses/lib)

add_executable (jones-debugger debugger.cc main_screen.cc)

add_dependencies (jones-debugger ncurses)

target_link_libraries (jones-debugger cpu)
target_link_libraries (jones-debugger ${boost-lib}/libboost_program_options.a)
target_link_libraries (jones-debugger ${boost-lib}/libboost_filesystem.a)
target_link_libraries (jones-debugger ${ncurses-lib}/libncurses.a)

target_include_directories (jones-debugger PRIVATE ${boost-include})
target_include_directories (jones-debugger PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

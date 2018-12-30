cmake_minimum_required (VERSION 3.7)

# boost
set (boost-include   ${EXTERNAL_DIR}/boost/include)
set (boost-lib       ${EXTERNAL_DIR}/boost/lib)

# ncurses
set (ncurses-include ${EXTERNAL_DIR}/ncurses/out/include)
set (ncurses-lib     ${EXTERNAL_DIR}/ncurses/out/lib)

add_executable (jones-debugger debugger.cc screen.cc)

target_link_libraries (jones-debugger cpu)
target_link_libraries (jones-debugger ${boost-lib}/libboost_program_options.a)
target_link_libraries (jones-debugger ${boost-lib}/libboost_filesystem.a)
target_link_libraries (jones-debugger ${ncurses-lib}/libncurses.a)

target_include_directories (jones-debugger PRIVATE ${boost-include})
target_include_directories (jones-debugger PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

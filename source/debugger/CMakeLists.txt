cmake_minimum_required (VERSION 3.7)

# boost
set (boost-include   ${EXTERNAL_DIR}/boost/include)
set (boost-lib       ${EXTERNAL_DIR}/boost/lib)

# ncurses
set (ncurses-include ${EXTERNAL_DIR}/ncurses/out/include)
set (ncurses-lib     ${EXTERNAL_DIR}/ncurses/out/lib)

add_executable (debugger debugger.cc)

target_link_libraries (debugger cpu)
target_link_libraries (debugger ${boost-lib}/libboost_program_options.a)
target_link_libraries (debugger ${boost-lib}/libboost_filesystem.a)

target_include_directories (debugger PRIVATE ${boost-include})
target_include_directories (debugger PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

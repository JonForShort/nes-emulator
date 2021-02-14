cmake_minimum_required (VERSION 3.14)

set (boost-modules --with-log --with-program_options --with-filesystem --with-test --with-serialization --with-iostreams)

add_custom_target (boost
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/source
  COMMAND ./bootstrap.sh --prefix=${DIR_PROJECT_OUT}
  COMMAND ./b2 link=static cxxflags=-fPIC linkflags=-pie ${boost-modules} install)

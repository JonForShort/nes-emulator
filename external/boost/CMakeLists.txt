cmake_minimum_required (VERSION 3.7)

add_custom_target (boost
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/source
  COMMAND ./bootstrap.sh --prefix=${DIR_PROJECT_OUT}
  COMMAND ./b2 variant=release link=static cxxflags='-fPIC' linkflags='-pie -nostartfiles' --with-log --with-program_options --with-filesystem --with-test --with-serialization install)

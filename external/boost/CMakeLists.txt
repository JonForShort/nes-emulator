cmake_minimum_required (VERSION 3.14)

set (boost-modules --with-log --with-program_options --with-filesystem --with-test --with-serialization --with-iostreams)

add_custom_target (boost
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/source
  COMMAND ./bootstrap.sh --prefix=${DIR_PROJECT_OUT}
  COMMAND ./b2 link=static cxxflags=-fPIC linkflags=-pie ${boost-modules} install
  BYPRODUCTS ${DIR_PROJECT_OUT}/lib/libboost_unit_test_framework.a
             ${DIR_PROJECT_OUT}/lib/libboost_log.a
             ${DIR_PROJECT_OUT}/lib/libboost_log_setup.a
             ${DIR_PROJECT_OUT}/lib/libboost_thread.a
             ${DIR_PROJECT_OUT}/lib/libboost_filesystem.a
             ${DIR_PROJECT_OUT}/lib/libboost_program_options.a
             ${DIR_PROJECT_OUT}/lib/libboost_serialization.a
             ${DIR_PROJECT_OUT}/lib/libboost_iostreams.a
)

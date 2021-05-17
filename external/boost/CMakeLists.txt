cmake_minimum_required(VERSION 3.14)

if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  set(boost-cxx-compiler-id "clang")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  set(boost-cxx-compiler-id "gcc")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Intel")
  set(boost-cxx-compiler-id "intel")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
  set(boost-cxx-compiler-id "msvc")
else()
  message(FATAL_ERROR "building boost with unsupported compiler")
endif()

set(boost-modules --with-log --with-program_options --with-filesystem --with-test --with-serialization --with-iostreams)
set(boost-flags toolset=${boost-cxx-compiler-id}-custom link=static threading=multi cxxflags=-fPIC linkflags=-pie headers)
set(boost-zlib -sZLIB_SOURCE=${DIR_ROOT_EXTERNAL}/zlib/source -sZLIB_INCLUDE=${DIR_ROOT_OUT}/zlib/include
               -sZLIB_LIBPATH=${DIR_ROOT_OUT}/zlib/lib)

add_custom_target(
  boost
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/source
  COMMAND ./bootstrap.sh --prefix=${DIR_PROJECT_OUT}
  COMMAND echo 'using ${boost-cxx-compiler-id} : custom : ${CXX} \;' >> ${CMAKE_CURRENT_SOURCE_DIR}/source/project-config.jam
  COMMAND ./b2 --build-dir=${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_SYSTEM_PROCESSOR} ${boost-flags} ${boost-modules} ${boost-zlib} install
  BYPRODUCTS ${DIR_PROJECT_OUT}/lib/libboost_unit_test_framework.a
             ${DIR_PROJECT_OUT}/lib/libboost_log.a
             ${DIR_PROJECT_OUT}/lib/libboost_log_setup.a
             ${DIR_PROJECT_OUT}/lib/libboost_thread.a
             ${DIR_PROJECT_OUT}/lib/libboost_filesystem.a
             ${DIR_PROJECT_OUT}/lib/libboost_program_options.a
             ${DIR_PROJECT_OUT}/lib/libboost_serialization.a
             ${DIR_PROJECT_OUT}/lib/libboost_iostreams.a)

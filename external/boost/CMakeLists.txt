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

set(boost-modules
    --with-log
    --with-program_options
    --with-filesystem
    --with-test
    --with-serialization
    --with-iostreams
    --with-context
    --with-regex
    --with-system
    --with-thread)
set(boost-flags toolset=${boost-cxx-compiler-id}-custom link=static threading=multi cxxflags=-fPIC linkflags=-pie headers)
set(boost-zlib -sZLIB_INCLUDE=${DIR_ROOT_OUT}/zlib/include -sZLIB_LIBPATH=${DIR_ROOT_OUT}/zlib/lib
               -sZLIB_SOURCE=${DIR_ROOT_EXTERNAL}/zlib/source)
set(boost-project-config ${CMAKE_CURRENT_BINARY_DIR}/project-config.jam)
set(boost-build-stamp ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_SYSTEM_PROCESSOR}/boost-build.stamp)

file(WRITE ${boost-project-config} "using ${boost-cxx-compiler-id} : custom : ${CXX} ;\n")

add_custom_command(
  OUTPUT ${boost-build-stamp}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/source
  COMMAND ./bootstrap.sh --prefix=${DIR_PROJECT_OUT}
  COMMAND ./b2 --build-dir=${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_SYSTEM_PROCESSOR} --user-config=${boost-project-config} ${boost-flags}
          ${boost-modules} ${boost-zlib} install
  COMMAND ${CMAKE_COMMAND} -E touch ${boost-build-stamp}
  BYPRODUCTS ${DIR_PROJECT_OUT}/lib/libboost_unit_test_framework.a
             ${DIR_PROJECT_OUT}/lib/libboost_log.a
             ${DIR_PROJECT_OUT}/lib/libboost_log_setup.a
             ${DIR_PROJECT_OUT}/lib/libboost_thread.a
             ${DIR_PROJECT_OUT}/lib/libboost_filesystem.a
             ${DIR_PROJECT_OUT}/lib/libboost_program_options.a
             ${DIR_PROJECT_OUT}/lib/libboost_serialization.a
             ${DIR_PROJECT_OUT}/lib/libboost_iostreams.a)

add_custom_target(boost DEPENDS ${boost-build-stamp})

add_custom_target(
  boost-zlib
  DEPENDS zlib
  COMMAND ${CMAKE_COMMAND} -E rename ${DIR_ROOT_EXTERNAL}/zlib/source/zconf.h.included ${DIR_ROOT_EXTERNAL}/zlib/source/zconf.h || true)

add_dependencies(boost boost-zlib)

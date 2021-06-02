cmake_minimum_required(VERSION 3.14)

include(ExternalProject)

ExternalProject_Add(
  gzip
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/source
  CONFIGURE_COMMAND CC=${CC} CXX=${CXX} ${CMAKE_CURRENT_SOURCE_DIR}/source/configure --prefix=${DIR_PROJECT_OUT} --host=${ARCH_NAME}
  BUILD_COMMAND ${MAKE})

add_custom_target(
  gzip_autoreconf
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/source
  COMMAND autoreconf -vfi)

add_dependencies(gzip gzip_autoreconf)

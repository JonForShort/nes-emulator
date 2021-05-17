cmake_minimum_required(VERSION 3.14)

include(ExternalProject)

ExternalProject_Add(
  ncurses
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/source
  BINARY_DIR ${DIR_PROJECT_BUILD}
  CONFIGURE_COMMAND CC=${CC} CXX=${CXX} ${CMAKE_CURRENT_SOURCE_DIR}/source/configure --prefix=${DIR_PROJECT_OUT} --host=${ARCH_NAME}
  BUILD_COMMAND ${MAKE})

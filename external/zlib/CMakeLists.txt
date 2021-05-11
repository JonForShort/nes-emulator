cmake_minimum_required (VERSION 3.14)

include (ExternalProject)

ExternalProject_Add (
  zlib
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/source
  CONFIGURE_COMMAND CC=${JONES_C_COMPILER} CXX=${JONES_CXX_COMPILER} ${CMAKE_CURRENT_SOURCE_DIR}/source/configure --prefix=${DIR_PROJECT_OUT}
  BUILD_COMMAND ${MAKE}
  BYPRODUCTS ${DIR_PROJECT_OUT}/lib/libz.a
)

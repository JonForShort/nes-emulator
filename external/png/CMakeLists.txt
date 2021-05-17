cmake_minimum_required(VERSION 3.14)

include(ExternalProject)

ExternalProject_Add(
  png
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/source
  BUILD_BYPRODUCTS ${DIR_PROJECT_OUT}/lib/libpng.a
  BINARY_DIR ${DIR_PROJECT_BUILD}
  CONFIGURE_COMMAND CC=${CC} CXX=${CXX} ${CMAKE_CURRENT_SOURCE_DIR}/source/configure --prefix=${DIR_PROJECT_OUT} --host=${ARCH_NAME}
                    --with-zlib-prefix=${DIR_ROOT_OUT}/zlib)

add_dependencies(png zlib)

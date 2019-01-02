cmake_minimum_required (VERSION 3.7)

include (ExternalProject)

ExternalProject_Add (
  sdl
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/source
  CONFIGURE_COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/source/configure --prefix=${DIR_PROJECT_OUT}
  BUILD_COMMAND ${MAKE}
)

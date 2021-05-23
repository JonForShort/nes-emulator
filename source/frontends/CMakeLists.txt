cmake_minimum_required (VERSION 3.7)

message ("BUILD_FRONTEND set to [" ${BUILD_FRONT_END} "]")

if (${BUILD_FRONTEND} MATCHES "SDL")

  message ("building SDL frontend")

  add_subdirectory (sdl)

endif ()

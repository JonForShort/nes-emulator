cmake_minimum_required (VERSION 3.7)

add_custom_target (gsl-headers COMMAND ${CMAKE_COMMAND}
  -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/source/include ${DIR_PROJECT_OUT}/include)

add_custom_target (gsl DEPENDS gsl-headers)

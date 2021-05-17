cmake_minimum_required(VERSION 3.14)

if(ANDROID)
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/platform/Android.cmake)
else()
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/platform/Default.cmake)
endif()

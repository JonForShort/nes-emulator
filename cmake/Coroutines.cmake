cmake_minimum_required(VERSION 3.14)

add_library(coroutines INTERFACE)

if(ANDROID)
  target_link_options(coroutines INTERFACE "-fcoroutines-ts")
else()
  target_link_options(coroutines INTERFACE "-fcoroutines")
endif()

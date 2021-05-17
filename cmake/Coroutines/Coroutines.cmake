cmake_minimum_required(VERSION 3.14)

add_library(coroutines INTERFACE)

if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  target_compile_options(coroutines INTERFACE "-fcoroutines-ts")
  target_compile_definitions(coroutines INTERFACE USE_COROUTINES_EXPERIMENTAL=1)
else()
  target_compile_options(coroutines INTERFACE "-fcoroutines")
  target_compile_definitions(coroutines INTERFACE USE_COROUTINES_STANDARD=1)
endif()

target_include_directories(coroutines INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Coroutines/include)

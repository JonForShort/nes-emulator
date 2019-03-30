cmake_minimum_required (VERSION 3.7)

set (source-files
  debugger.cc
)

add_library (jones-debugger SHARED ${source-files})

target_link_libraries (jones-debugger cpu)
target_link_libraries (jones-debugger log)

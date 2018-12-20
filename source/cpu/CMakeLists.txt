cmake_minimum_required (VERSION 3.7)

include_directories (include)

set (sources cpu.cc)

add_library (cpu STATIC ${sources})

add_executable (cpu_test cpu_test.cc)

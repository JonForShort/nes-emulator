cmake_minimum_required (VERSION 3.7)

# boost
set (boost-include ${DIR_ROOT_OUT}/boost/include)
set (boost-lib     ${DIR_ROOT_OUT}/boost/lib)

set (sources cpu_test.cc)

add_executable (cpu_test ${sources})

add_dependencies (cpu_test capstone)

target_link_libraries (cpu_test cpu)
target_link_libraries (cpu_test ${boost-lib}/libboost_unit_test_framework.a)

target_include_directories (cpu_test PRIVATE ${boost-include})

add_test(NAME cpu_test COMMAND cpu_test)

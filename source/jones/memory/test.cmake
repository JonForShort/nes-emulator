cmake_minimum_required (VERSION 3.7)

# boost
set (boost-include ${DIR_ROOT_OUT}/boost/include)
set (boost-lib     ${DIR_ROOT_OUT}/boost/lib)

set (sources memory_test.cc)

add_executable (memory_test ${sources})

target_link_libraries (memory_test memory)
target_link_libraries (memory_test ${boost-lib}/libboost_unit_test_framework.a)

target_include_directories (memory_test PRIVATE ${boost-include})

add_test(NAME memory_test COMMAND memory_test)

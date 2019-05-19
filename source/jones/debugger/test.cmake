cmake_minimum_required (VERSION 3.7)

# boost

set (boost-include ${DIR_ROOT_OUT}/boost/include)
set (boost-lib     ${DIR_ROOT_OUT}/boost/lib)

set (sources debugger_test.cc)

add_executable (debugger_test ${sources})

target_link_libraries (debugger_test debugger)
target_link_libraries (debugger_test ${boost-lib}/libboost_unit_test_framework.a)

target_include_directories (debugger_test PRIVATE ${boost-include})

add_test(NAME debugger_test COMMAND debugger_test)

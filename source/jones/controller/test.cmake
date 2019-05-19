cmake_minimum_required (VERSION 3.7)

# boost
set (boost-include ${DIR_ROOT_OUT}/boost/include)
set (boost-lib     ${DIR_ROOT_OUT}/boost/lib)

set (sources controller_test.cc)

add_executable (controller_test ${sources})

target_link_libraries (controller_test controller)
target_link_libraries (controller_test ${boost-lib}/libboost_unit_test_framework.a)

target_include_directories (controller_test PRIVATE ${boost-include})

add_test(NAME controller_test COMMAND controller_test)

cmake_minimum_required(VERSION 3.14)

# boost
set(boost-include ${DIR_ROOT_OUT}/boost/include)
set(boost-lib ${DIR_ROOT_OUT}/boost/lib)

set(sources configuration_test.cc)

add_executable(configuration_test ${sources})

add_dependencies(configuration_test boost)

target_link_libraries(configuration_test configuration)
target_link_libraries(configuration_test ${boost-lib}/libboost_unit_test_framework.a)

target_include_directories(configuration_test PRIVATE ${boost-include})

add_test(NAME configuration_test COMMAND configuration_test)

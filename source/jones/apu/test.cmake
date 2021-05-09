cmake_minimum_required(VERSION 3.14)

# boost
set(boost-include ${DIR_ROOT_OUT}/boost/include)
set(boost-lib ${DIR_ROOT_OUT}/boost/lib)

set(sources apu_test.cc)

add_executable(apu_test ${sources})

add_dependencies(apu_test boost)

target_link_libraries(apu_test apu)
target_link_libraries(apu_test ${boost-lib}/libboost_unit_test_framework.a)

target_include_directories(apu_test PRIVATE ${boost-include})

add_test(NAME apu_test COMMAND apu_test)

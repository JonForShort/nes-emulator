cmake_minimum_required(VERSION 3.14)

# boost
set(boost-include ${DIR_ROOT_OUT}/boost/include)
set(boost-lib ${DIR_ROOT_OUT}/boost/lib)

set(sources screen_test.cc)

add_executable(screen_test ${sources})

add_dependencies(screen_test boost)

target_link_libraries(screen_test screen)
target_link_libraries(screen_test ${boost-lib}/libboost_unit_test_framework.a)

target_include_directories(screen_test PRIVATE ${boost-include})

add_test(NAME screen_test COMMAND screen_test)

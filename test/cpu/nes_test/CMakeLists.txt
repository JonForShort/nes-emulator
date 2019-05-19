cmake_minimum_required (VERSION 3.7)

# boost

set (boost-include ${DIR_ROOT_OUT}/boost/include)
set (boost-lib     ${DIR_ROOT_OUT}/boost/lib)

# nes_test

set (sources nes_test.cc)

add_executable (test_suite_nes_test ${sources})

target_link_libraries (test_suite_nes_test log)
target_link_libraries (test_suite_nes_test jones-lib)
target_link_libraries (test_suite_nes_test ${boost-lib}/libboost_unit_test_framework.a)
target_link_libraries (test_suite_nes_test ${boost-lib}/libboost_filesystem.a)

target_include_directories (test_suite_nes_test PRIVATE ${boost-include})

add_test(NAME test_suite_nes_test COMMAND test_suite_nes_test ${CMAKE_CURRENT_SOURCE_DIR}/nes_test.nes ${CMAKE_CURRENT_SOURCE_DIR}/nes_test.log)

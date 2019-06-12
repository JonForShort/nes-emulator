cmake_minimum_required (VERSION 3.7)

# boost

set (boost-include ${DIR_ROOT_OUT}/boost/include)
set (boost-lib     ${DIR_ROOT_OUT}/boost/lib)

# nes_test

set (sources cpu_interrupts_v2.cc)

add_executable (test_suite_cpu_interrupts_v2 ${sources})

target_link_libraries (test_suite_cpu_interrupts_v2 log)
target_link_libraries (test_suite_cpu_interrupts_v2 jones-lib)
target_link_libraries (test_suite_cpu_interrupts_v2 ${boost-lib}/libboost_unit_test_framework.a)
target_link_libraries (test_suite_cpu_interrupts_v2 ${boost-lib}/libboost_filesystem.a)

target_include_directories (test_suite_cpu_interrupts_v2 PRIVATE ${boost-include})

add_test(NAME test_suite_cpu_interrupts_v2 COMMAND test_suite_cpu_interrupts_v2 ${CMAKE_CURRENT_SOURCE_DIR}/cpu_interrupts.nes)

cmake_minimum_required (VERSION 3.7)

set (boost-include ${EXTERNAL_DIR}/boost/include)
set (boost-lib     ${EXTERNAL_DIR}/boost/lib)

set (sources cpu.cc)

add_library (cpu STATIC ${sources})

target_include_directories (cpu PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable (cpu_test cpu_test.cc)

target_link_libraries (cpu_test ${boost-lib}/libboost_program_options.a)

target_include_directories (cpu_test PRIVATE ${boost-include})
target_include_directories (cpu_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

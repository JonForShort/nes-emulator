cmake_minimum_required (VERSION 3.7)

# boost
set (boost-include ${DIR_ROOT_OUT}/boost/include)
set (boost-lib     ${DIR_ROOT_OUT}/boost/lib)

set (sources ppu_test.cc)

add_executable (ppu_test ${sources})

target_link_libraries (ppu_test ppu)
target_link_libraries (ppu_test ${boost-lib}/libboost_unit_test_framework.a)

target_include_directories (ppu_test PRIVATE ${boost-include})

add_test(NAME ppu_test COMMAND ppu_test)

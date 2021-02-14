cmake_minimum_required (VERSION 3.14)

# boost
set (boost-include ${DIR_ROOT_OUT}/boost/include)
set (boost-lib     ${DIR_ROOT_OUT}/boost/lib)

set (sources cartridge_test.cc)

add_executable (cartridge_test ${sources})

target_link_libraries (cartridge_test cartridge)
target_link_libraries (cartridge_test ${boost-lib}/libboost_unit_test_framework.a)

target_include_directories (cartridge_test PRIVATE ${boost-include})

add_test(NAME cartridge_test COMMAND cartridge_test)

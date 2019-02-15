cmake_minimum_required (VERSION 3.7)

# boost
set (boost-include   ${DIR_ROOT_OUT}/boost/include)
set (boost-lib       ${DIR_ROOT_OUT}/boost/lib)

set (source-files
  tool.cc
)

add_executable (jones-tool ${source-files})

add_dependencies (jones-tool boost)

target_link_libraries (jones-tool cpu)
target_link_libraries (jones-tool cartridge)
target_link_libraries (jones-tool ${boost-lib}/libboost_program_options.a)
target_link_libraries (jones-tool ${boost-lib}/libboost_filesystem.a)

target_include_directories (jones-tool PRIVATE ${boost-include})
target_include_directories (jones-tool PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)


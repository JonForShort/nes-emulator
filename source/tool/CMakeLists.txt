cmake_minimum_required (VERSION 3.7)

# boost

set (boost-include   ${DIR_ROOT_OUT}/boost/include)
set (boost-lib       ${DIR_ROOT_OUT}/boost/lib)

# building tool library

add_library(jones-tool-lib STATIC tool_dump.cc)

add_custom_target (jones-tool-lib-copy-headers COMMAND ${CMAKE_COMMAND}
        -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/include ${DIR_ROOT_OUT}/jones/include/jones-tool)

add_dependencies(jones-tool-lib jones-tool-lib-copy-headers)

target_link_libraries (jones-tool-lib cpu)
target_link_libraries (jones-tool-lib cartridge)
target_link_libraries (jones-tool-lib ${boost-lib}/libboost_filesystem.a)

target_include_directories (jones-tool-lib PRIVATE ${boost-include})
target_include_directories (jones-tool-lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# building tool executable

add_executable (jones-tool tool.cc)

target_link_libraries (jones-tool jones-tool-lib)
target_link_libraries (jones-tool ${boost-lib}/libboost_program_options.a)
target_link_libraries (jones-tool ${boost-lib}/libboost_filesystem.a)

target_include_directories (jones-tool PRIVATE ${boost-include})

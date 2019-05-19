cmake_minimum_required (VERSION 3.7)

# boost

set (boost-include   ${DIR_ROOT_OUT}/boost/include)
set (boost-lib       ${DIR_ROOT_OUT}/boost/lib)

# png

set (png-include     ${DIR_ROOT_OUT}/png/include)
set (png-lib         ${DIR_ROOT_OUT}/png/lib)

# zlib

set (zlib-lib        ${DIR_ROOT_OUT}/zlib/lib)

# building tool library

add_library(jones-tool-lib STATIC tool_dump.cc tool_decoder.cc image_writer.cc)

add_custom_target (jones-tool-lib-copy-headers COMMAND ${CMAKE_COMMAND}
        -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/include ${DIR_ROOT_OUT}/jones/include/jones-tool)

add_dependencies(jones-tool-lib jones-tool-lib-copy-headers)

target_link_libraries (jones-tool-lib cpu)
target_link_libraries (jones-tool-lib cartridge)
target_link_libraries (jones-tool-lib ${boost-lib}/libboost_filesystem.a)
target_link_libraries (jones-tool-lib ${boost-lib}/libboost_serialization.a)
target_link_libraries (jones-tool-lib ${png-lib}/libpng.a)
target_link_libraries (jones-tool-lib ${zlib-lib}/libz.a)

target_include_directories (jones-tool-lib PRIVATE ${boost-include})
target_include_directories (jones-tool-lib PRIVATE ${png-include})

target_include_directories (jones-tool-lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# building tool executable

add_executable (jones-tool tool.cc)

target_link_libraries (jones-tool jones-tool-lib)
target_link_libraries (jones-tool ${boost-lib}/libboost_program_options.a)
target_link_libraries (jones-tool ${boost-lib}/libboost_filesystem.a)

target_include_directories (jones-tool PRIVATE ${boost-include})

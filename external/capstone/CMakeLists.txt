cmake_minimum_required(VERSION 3.14)

option(CAPSTONE_BUILD_TESTS "" OFF)

add_subdirectory(source)

add_custom_target(capstone-headers COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/source/include
                                           ${DIR_PROJECT_OUT}/include)

add_custom_target(capstone DEPENDS capstone-static capstone-shared capstone-headers)

cmake_minimum_required(VERSION 3.14)

# boost
set(boost-include ${DIR_ROOT_OUT}/boost/include)
set(boost-lib ${DIR_ROOT_OUT}/boost/lib)

# cppcoro
set(cppcoro-include ${DIR_ROOT_OUT}/cppcoro/include)
set(cppcoro-lib ${DIR_ROOT_OUT}/cppcoro/lib)

# gsl
set(gsl-include ${DIR_ROOT_OUT}/gsl/include)

set(sources cpu.cc decode.cc disassemble.cc instruction.cc status_register.cc)

add_library(cpu STATIC ${sources})

add_dependencies(cpu boost)
add_dependencies(cpu cppcoro)
add_dependencies(cpu gsl)

target_link_libraries(cpu configuration)
target_link_libraries(cpu log)
target_link_libraries(cpu memory)
target_link_libraries(cpu coroutines)

target_include_directories(cpu PRIVATE ${boost-include})
target_include_directories(cpu PRIVATE ${cppcoro-include})
target_include_directories(cpu PRIVATE ${gsl-include})
target_include_directories(cpu PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

include(${CMAKE_CURRENT_SOURCE_DIR}/test.cmake)

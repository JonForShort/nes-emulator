cmake_minimum_required(VERSION 3.14)

set(boost-include ${DIR_ROOT_OUT}/boost/include)
set(boost-lib ${DIR_ROOT_OUT}/boost/lib)

set(sources log.cc)

add_library(log STATIC ${sources})

add_dependencies(log boost)

target_link_libraries(log ${boost-lib}/libboost_log.a)
target_link_libraries(log ${boost-lib}/libboost_log_setup.a)
target_link_libraries(log ${boost-lib}/libboost_thread.a)
target_link_libraries(log pthread)

target_include_directories(log PRIVATE ${boost-include})

target_include_directories(log PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

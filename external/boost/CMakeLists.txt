cmake_minimum_required (VERSION 3.7)

file (MAKE_DIRECTORY ${DIR_PROJECT_OUT})

file (COPY ${CMAKE_CURRENT_SOURCE_DIR}/lib     DESTINATION ${DIR_PROJECT_OUT})
file (COPY ${CMAKE_CURRENT_SOURCE_DIR}/include DESTINATION ${DIR_PROJECT_OUT})

#
# place-holder for when real target exists.
#
add_custom_target (boost)

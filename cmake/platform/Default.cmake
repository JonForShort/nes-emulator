set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/out/default/${CMAKE_PROJECT_NAME}/lib)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/out/default/${CMAKE_PROJECT_NAME}/lib)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/out/default/${CMAKE_PROJECT_NAME}/bin)

set(JONES_DIR_ROOT_OUT ${CMAKE_CURRENT_SOURCE_DIR}/out)

set(EXTERNAL_PROJECTS
    ncurses
    sndio
    sdl
    boost
    capstone
    gsl
    zlib
    png
    gzip
    cppcoro)

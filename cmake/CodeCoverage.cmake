if ("$ENV{CODE_COVERAGE}" STREQUAL "1")
  set(JONES_CXX_FLAGS_DEBUG "${JONES_CXX_FLAGS_DEBUG} -fprofile-arcs -ftest-coverage --coverage")
endif()


if("$ENV{ADDRESS_SANITIZER}" STREQUAL "1")

  set(JONES_CXX_FLAGS_DEBUG "${JONES_CXX_FLAGS_DEBUG} -fsanitize=address -fno-omit-frame-pointer")

  set(JONES_EXE_LINKER_FLAGS_DEBUG "${JONES_EXE_LINKER_FLAGS_DEBUG} -fsanitize=address -fno-omit-frame-pointer")

  set(JONES_MODULE_LINKER_FLAGS_DEBUG "${JONES_MODULE_LINKER_FLAGS_DEBUG} -fsanitize=address -fno-omit-frame-pointer")

  set(JONES_SHARED_LINKER_FLAGS_DEBUG "${JONES_SHARED_LINKER_FLAGS_DEBUG} -fsanitize=address -fno-omit-frame-pointer")

  set(JONES_STATIC_LINKER_FLAGS_DEBUG "${JONES_STATIC_LINKER_FLAGS_DEBUG} -fsanitize=address -fno-omit-frame-pointer")

endif()

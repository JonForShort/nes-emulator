if(ANDROID)

  set(JONES_DIR_ROOT_OUT ${CMAKE_CURRENT_SOURCE_DIR}/out/android/${CMAKE_BUILD_TYPE}/${CMAKE_ANDROID_ARCH_ABI})

  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${JONES_DIR_ROOT_OUT}/${CMAKE_PROJECT_NAME}/lib)

  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${JONES_DIR_ROOT_OUT}/${CMAKE_PROJECT_NAME}/lib)

  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${JONES_DIR_ROOT_OUT}/${CMAKE_PROJECT_NAME}/bin)

  set(EXTERNAL_PROJECTS
      boost
      capstone
      gsl
      zlib
      png
      gzip
      cppcoro)

  if(ANDROID_ABI STREQUAL armeabi-v7a)

    set(JONES_COMPILER_TARGET "armv7a-linux-androideabi${ANDROID_NATIVE_API_LEVEL}")

  elseif(ANDROID_ABI STREQUAL arm64-v8a)

    set(JONES_COMPILER_TARGET "aarch64-linux-android${ANDROID_NATIVE_API_LEVEL}")

  elseif(ANDROID_ABI STREQUAL x86)

    set(JONES_COMPILER_TARGET "i686-linux-android${ANDROID_NATIVE_API_LEVEL}")

  elseif(ANDROID_ABI STREQUAL x86_64)

    set(JONES_COMPILER_TARGET "x86_64-linux-android${ANDROID_NATIVE_API_LEVEL}")

  endif()

  set(JONES_C_COMPILER ${ANDROID_TOOLCHAIN_ROOT}/bin/${JONES_COMPILER_TARGET}-clang)

  set(JONES_CXX_COMPILER ${ANDROID_TOOLCHAIN_ROOT}/bin/${JONES_COMPILER_TARGET}-clang++)

endif()

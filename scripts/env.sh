#!/usr/bin/env bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

JONES_OUT_DIR=${SCRIPT_DIR}/../out/jones

JONES_BUILD_DIR=${SCRIPT_DIR}/../build

JONES_CMAKE_DIR=${SCRIPT_DIR}/../cmake

jones() {

    JONES=${JONES_OUT_DIR}/bin/jones

    if [ ! -f "${JONES}" ]; then
        echo "error: unable to find jones; path is not valid [${JONES}]"
        return -1
    fi

    ${JONES} "$@"
}

jones_asan() {
    
    ASAN_OPTIONS="fast_unwind_on_malloc=0" \
    LSAN_OPTIONS="suppressions=${JONES_CMAKE_DIR}/AddressSanitizer.suppressions:verbosity=1:log_threads=1" \
    jones "$@"
}

jones_tool() {

    JONES_TOOL=${JONES_OUT_DIR}/bin/jones-tool

    if [ ! -f "${JONES_TOOL}" ]; then
        echo "error: unable to find tool; path is not valid [${JONES_TOOL}]"
        return -1
    fi

    ${JONES_TOOL} "$@"
}

jones_build() {

    mkdir -p ${JONES_BUILD_DIR}

    pushd ${JONES_BUILD_DIR}
        cmake -DCMAKE_BUILD_TYPE=Debug ..
        make "$@"
    popd
}

jones_test() {

    jones_build "$@"

    pushd ${JONES_BUILD_DIR}
        make test
    popd
}

jones_test_cm() {

    IS_CM_BUILD=1 jones_test "$@"
}

jones_test_asan() {

    ADDRESS_SANITIZER=1 jones_test "$@"
}

jones_test_code_coverage() {

    CODE_COVERAGE=1 jones_test "$@"

    if ! [ -x "$(command -v gcovr)" ]; then
        echo "error: unable to find gcovr tool; tool is required to generate code coverage"
        exit 1
    fi

    CODE_COVERAGE_FILE=${JONES_BUILD_DIR}/coverage.html

    gcovr -r ${SCRIPT_DIR}/.. --html --html-details -o ${CODE_COVERAGE_FILE}

    echo ""
    echo "successfully generated code coverage"
    echo ""
    echo ${CODE_COVERAGE_FILE}
    echo ""
}

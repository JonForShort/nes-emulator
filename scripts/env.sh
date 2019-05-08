#!/usr/bin/env bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

JONES_OUT_DIR=${SCRIPT_DIR}/../out/jones

JONES_BUILD_DIR=${SCRIPT_DIR}/../build

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

jones_test_code_coverage() {

    CODE_COVERAGE=1 jones_test
}

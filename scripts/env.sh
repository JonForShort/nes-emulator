#!/usr/bin/env bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
OUT_DIR="${SCRIPT_DIR}/../out/jones"

jones_tool() {

    JONES_TOOL=${OUT_DIR}/bin/jones-tool
    if [ ! -f "${JONES_TOOL}" ]; then
	echo "error: unable to find tool; path is not valid [${JONES_TOOL}]"
	return -1
    fi

    ${JONES_TOOL} "$@"
}

jones_build() {

    JONES_BUILD_DIR=${SCRIPT_DIR}/../build

    mkdir -p ${JONES_BUILD_DIR}

    pushd ${JONES_BUILD_DIR}
        cmake -DCMAKE_BUILD_TYPE=Debug ..
        make "$@" && make test
    popd
}

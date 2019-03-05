#!/usr/bin/env bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
OUT_DIR="${SCRIPT_DIR}/../out/jones"

jones_debugger() {

    JONES_DEBUGGER=${OUT_DIR}/bin/jones-debugger
    if [ ! -f "${JONES_DEBUGGER}" ]; then
	echo "error: unable to find debugger; path is not valid [${JONES_DEBUGGER}]"
	return -1
    fi

    ${JONES_DEBUGGER} "$@"
}

jones_build() {

    JONES_BUILD_DIR=${SCRIPT_DIR}/../build

    mkdir -p ${JONES_BUILD_DIR}

    pushd ${JONES_BUILD_DIR}
        cmake ..
        make && make test
    popd
}

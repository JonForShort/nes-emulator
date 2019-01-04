#!/usr/bin/env bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
OUT_DIR="${SCRIPT_DIR}/../out/jones"

jones_debugger() {

    JONES_DEBUGGER=${OUT_DIR}/bin/jones-debugger
    if [ ! -f "${JONES_DEBUGGER}" ]; then
	echo "error: unable to find debugger; path is not valid [${JONES_DEBUGGER}]"
	return -1
    fi

    DEBUG_FILE=$1
    if [ ! -f "${DEBUG_FILE}" ]; then
	echo "error: must provide file to debug; path is not valid [${DEBUG_FILE}]"
	return -2
    fi

    ${JONES_DEBUGGER} -g -f ${DEBUG_FILE}	
}

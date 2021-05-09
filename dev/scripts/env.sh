#!/usr/bin/env bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

JONES_ROOT_DIR=${SCRIPT_DIR}/../..

JONES_OUT_DIR=${JONES_ROOT_DIR}/out/jones

JONES_BUILD_DIR=${JONES_ROOT_DIR}/build

JONES_CMAKE_DIR=${JONES_ROOT_DIR}/cmake

JONES_EXTERNAL_DIR=${JONES_ROOT_DIR}/external

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
        cmake -DBUILD_FRONTEND=SDL -DCMAKE_BUILD_TYPE=Debug ..
        make "$@"
    popd
}

jones_build_cm() {

    pushd ${JONES_ROOT_DIR}

    COMPOSE_DOCKER_CLI_BUILD=1 DOCKER_BUILDKIT=1 docker-compose build

    COMPOSE_DOCKER_CLI_BUILD=1 DOCKER_BUILDKIT=1 docker-compose run jones ./dev/scripts/env.sh jones_build

    popd
}

jones_test() {

    jones_build "$@"

    pushd ${JONES_BUILD_DIR}
        make test
    popd
}

jones_test_cm() {

    pushd ${JONES_ROOT_DIR}

    COMPOSE_DOCKER_CLI_BUILD=1 DOCKER_BUILDKIT=1 docker-compose build

    COMPOSE_DOCKER_CLI_BUILD=1 DOCKER_BUILDKIT=1 docker-compose run -e IS_CM_BUILD=1 jones ./dev/scripts/env.sh jones_test "$@"

    popd
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

jones_setup_environment() {

    echo JONES_UID=$(id -u $USER):$(id -g $USER) > .env
}

jones_update_submodules() {(

    git submodule update --recursive --remote
)}

jones_setup_vscode() {

    mkdir -p ${JONES_ROOT_DIR}/.vscode

    pushd ${JONES_ROOT_DIR}/.vscode

    ln -s $(realpath --relative-to=${JONES_ROOT_DIR}/.vscode ${JONES_ROOT_DIR}/dev/vscode)/launch.json launch.json

    ln -s $(realpath --relative-to=${JONES_ROOT_DIR}/.vscode ${JONES_ROOT_DIR}/dev/vscode)/tasks.json tasks.json

    popd
}

jones_format_cmake() {

    for i in $(find ${JONES_ROOT_DIR}/cmake \
                    ${JONES_ROOT_DIR}/source/jones \
                    ${JONES_ROOT_DIR}/source/tools \
                    -type f \( -iname '*.cmake' -o -iname 'CMakeLists.txt' \)); do
        cmake-format -i ${i}
    done
}

jones_setup_environment

"$@"

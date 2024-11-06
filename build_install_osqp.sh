#!/bin/bash

ROOT_DIR=$(pwd)

OSQP_ROOT=${ROOT_DIR}/src/uve_control/thirdparty/osqp

cd ${OSQP_ROOT}

if [ ! -d "build" ]; then
    mkdir build
fi
if [ ! -d "install" ]; then
    mkdir install
fi

cd build
cmake -DCMAKE_INSTALL_PREFIX=${OSQP_ROOT}/install ..
make -j4
make install

cd ${ROOT_DIR}
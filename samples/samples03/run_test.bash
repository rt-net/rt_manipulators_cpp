#!/usr/bin/env bash

set -e

BUILD_DIR=build

echo "samples03で使用するライブラリをテストします"
cd $(dirname $0)/test

if [ ! -d $BUILD_DIR ]; then
    mkdir $BUILD_DIR
fi

cd $BUILD_DIR
cmake ..
make
CTEST_OUTPUT_ON_FAILURE=1 ctest

echo "ライブラリをテストしました"
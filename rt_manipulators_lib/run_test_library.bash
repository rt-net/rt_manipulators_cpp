#!/usr/bin/env bash

set -e

BUILD_DIR=build

echo "ライブラリをテストします"
cd $(dirname $0)/test

if [ ! -d $BUILD_DIR ]; then
    mkdir $BUILD_DIR
fi

cd $BUILD_DIR
cmake ..
make
ctest

echo "ライブラリをテストしました"
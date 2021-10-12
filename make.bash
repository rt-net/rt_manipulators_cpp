#!/usr/bin/env bash

set -e

BUILD_DIR=build

echo "ビルドします"
cd $(dirname $0)

if [ ! -d $BUILD_DIR ]; then
    mkdir $BUILD_DIR
fi

cd $BUILD_DIR
cmake ..
make

echo "ビルドしました"

#!/usr/bin/env bash

set -e

BUILD_DIR=build

echo "サンプルプログラムをビルドします"
cd $(dirname $0)

if [ ! -d $BUILD_DIR ]; then
    mkdir $BUILD_DIR
fi

cd $BUILD_DIR
cmake ..
make

echo "サンプルプログラムをビルドしました"
#!/usr/bin/env bash

set -e

BUILD_DIR=build

echo "ライブラリをビルドします"
cd $(dirname $0)

if [ ! -d $BUILD_DIR ]; then
    mkdir $BUILD_DIR
fi

cd $BUILD_DIR
cmake ..
make

echo "ライブラリをビルドしました"

echo "ライブラリをインストールします"
sudo make install
# 共有ライブラリのリンクとキャッシュファイルの作成
sudo ldconfig
echo "ライブラリをインストールしました"

#!/usr/bin/env bash

set -e

BUILD_DIR=build

echo "ライブラリをアンインストールするため、次のファイルを削除します"

cat $BUILD_DIR/install_manifest.txt
echo ""
xargs sudo rm -rf < $BUILD_DIR/install_manifest.txt

echo "アンインストールしました"

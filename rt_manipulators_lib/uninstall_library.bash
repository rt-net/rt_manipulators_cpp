#!/usr/bin/env bash

set -e

BUILD_DIR=build

echo "ライブラリをアンインストールするため、次のファイルを削除します"

cd $(dirname $0)/$BUILD_DIR
cat install_manifest.txt
echo ""
xargs sudo rm -rf < install_manifest.txt

echo "アンインストールしました"

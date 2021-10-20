#!/usr/bin/env bash

set -e

cd $(dirname $0)

echo "cpplintを実行し、コードフォーマットをチェックします"
cpplint --filter=-build/c++11,-runtime/reference --extensions=hpp,cpp include/* src/*

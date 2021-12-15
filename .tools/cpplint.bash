#!/usr/bin/env bash

set -e

SCRIPT_DIR=$(dirname $0)
LIB_DIR=$SCRIPT_DIR/../rt_manipulators_lib
SAMPLES01_DIR=$SCRIPT_DIR/../samples/samples01
SAMPLES02_DIR=$SCRIPT_DIR/../samples/samples02

function cpplint_check () {
    cpplint --filter=-build/c++11,-runtime/reference --linelength=100 --extensions=hpp,cpp $1/include/* $1/src/* $1/test/*
}

echo "cpplintを実行し、コードフォーマットをチェックします"

cpplint_check $LIB_DIR
cpplint_check $SAMPLES01_DIR
cpplint_check $SAMPLES02_DIR


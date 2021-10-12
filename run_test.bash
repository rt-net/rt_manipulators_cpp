#!/usr/bin/env bash

set -e

$(dirname $0)/make.bash
cd $(dirname $0)/bin
./test_app

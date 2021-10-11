#!/usr/bin/bash -e

$(dirname $0)/make.bash
cd $(dirname $0)/bin
./test
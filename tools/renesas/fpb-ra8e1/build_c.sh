#!/bin/bash
set -e

cd ./nuttx
make distclean
./tools/configure.sh fpb-ra8e1:nsh
make -j$(nproc)
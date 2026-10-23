#!/bin/bash
set -e
export PATH=/usr/local/cuda/bin:$PATH
cd "$(dirname "$0")"
rm -rf build_asan
mkdir -p build_asan
cd build_asan
cmake .. -DCMAKE_BUILD_TYPE=Release -DENABLE_ASAN=ON
make -j6
ctest --output-on-failure

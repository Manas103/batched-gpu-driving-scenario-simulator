#!/bin/bash
set -e
export PATH=/usr/local/cuda/bin:$PATH
cd "$(dirname "$0")"
bash build.sh
cd build
ctest --output-on-failure
echo "--- bench_throughput ---"
./bench_throughput
echo "--- compare_gpu_cpu ---"
./compare_gpu_cpu

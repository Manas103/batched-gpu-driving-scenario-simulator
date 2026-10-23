#!/bin/bash
set -e
export PATH=/usr/local/cuda/bin:$PATH
cd "$(dirname "$0")"
python3 tools/run_replay_digests.py build --n 4096 --ticks 300 --sessions 100

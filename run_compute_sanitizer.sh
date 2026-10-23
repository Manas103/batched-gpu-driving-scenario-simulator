#!/bin/bash
set -e
export PATH=/usr/local/cuda/bin:$PATH
cd "$(dirname "$0")"
python3 tools/make_manifest.py 42 64 /tmp/small.manifest
compute-sanitizer --tool memcheck ./build/run_session /tmp/small.manifest 50

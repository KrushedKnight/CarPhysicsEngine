#!/bin/bash
mkdir -p build_web
cd build_web
emcmake cmake .. -DCMAKE_BUILD_TYPE=Release
emmake make
echo "Build complete. Output files in build_web/"
text changes

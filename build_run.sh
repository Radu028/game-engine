#!/bin/bash
set -e

echo "Building project..."
mkdir -p build
cd build

if ! cmake ..; then
    echo "CMake configuration failed!"
    exit 1
fi

if ! make -j$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4); then
    echo "Build failed!"
    exit 1
fi

echo "Build successful! Starting game..."
./GameProject

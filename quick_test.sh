#!/bin/bash
set -e

echo "Quick Test Script"
echo "=================="

mkdir -p build
cd build

echo "Building project..."
if ! make -j$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4); then
    echo "Build failed!"
    exit 1
fi

echo "Build successful!"
echo "Starting game (press ESC to exit)..."
echo ""
echo "Test checklist:"
echo "- Character should stand upright immediately"
echo "- Press SPACE to jump - should work!"
echo "- Use WASD to move around"
echo "- Character should remain stable"
echo ""

./GameProject

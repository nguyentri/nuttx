#!/usr/bin/env bash

# Clean, configure, and build for the given target
TARGET="$1"

if [ -z "$TARGET" ]; then
  echo "Usage: $0 <target_name>"
  echo "example: $0 fpb-ra8e1:nsh"
  exit 1
fi

# Clean previous CMake build
rm -rf build/

# Run configure.sh first
./tools/configure.sh $TARGET || { echo "[!] configure.sh failed"; exit 1; }

# Configure with CMake (always use build/ as per README)
cmake -B build -DBOARD_CONFIG=$TARGET -GNinja

# Build
cmake --build build

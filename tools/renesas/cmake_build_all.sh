#!/usr/bin/env bash

# Clean, configure, and build for the given target
TARGET="$1"

if [ -z "$TARGET" ]; then
  echo "Usage: $0 <target_name>"
  echo "example: $0 fpb-ra8e1:nsh"
  exit 1
fi

# Clean
rm -rf build/$TARGET

# Configure
cmake -Bbuild/$TARGET -DBOARD_CONFIG=$TARGET . -GNinja

# Build
cmake --build build/$TARGET

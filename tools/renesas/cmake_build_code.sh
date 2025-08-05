#!/usr/bin/env bash

# Build only for the given target (assumes already configured)
TARGET="$1"

if [ -z "$TARGET" ]; then
  echo "Usage: $0 <target_name>"
  echo "example: $0 fpb-ra8e1:nsh"
  exit 1
fi

# Run configure.sh first (to ensure config is present)
./tools/configure.sh $TARGET || { echo "[!] configure.sh failed"; exit 1; }

# Build (assumes CMake already configured, always use build/)
cmake --build build
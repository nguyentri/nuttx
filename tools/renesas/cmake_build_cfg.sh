#!/usr/bin/env bash

# Configure only for the given target (no build, no clean)
TARGET="$1"

if [ -z "$TARGET" ]; then
  echo "Usage: $0 <target_name>"
  echo "example: $0 fpb-ra8e1:nsh"
  exit 1
fi

cmake -Bbuild/$TARGET -DBOARD_CONFIG=$TARGET . -GNinja

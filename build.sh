#!/bin/bash

# Usage:
#   ./build.sh <package_name> [--clean]

set -e  # Exit on error

if [ -z "$1" ]; then
  echo "[ERROR] Package name required."
  echo "Usage: ./build.sh <package_name> [--clean]"
  exit 1
fi

PACKAGE_NAME="$1"
CLEAN_BUILD=false

if [ "$2" == "--clean" ]; then
  CLEAN_BUILD=true
fi

if $CLEAN_BUILD; then
  echo "[INFO] Cleaning previous build..."
  rm -rf build/ install/ log/
else
  echo "[INFO] Skipping clean. Reusing existing build artifacts."
fi

echo "[INFO] Building package: $PACKAGE_NAME"
colcon build \
  --packages-up-to "$PACKAGE_NAME" \
  --merge-install \
  --cmake-args -DCMAKE_INSTALL_ALWAYS=ON -DCMAKE_BUILD_TYPE=Release

echo "[INFO] Build finished for: $PACKAGE_NAME"
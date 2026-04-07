#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="${1:-$SCRIPT_DIR/build}"

cat <<EOF
Preparing a local pdpcFeature build in:
  $BUILD_DIR

Expected system dependencies:
  - CMake
  - Boost
  - CGAL

This helper only configures and builds the public executables.
It does not install packages automatically and it skips figure-generation
steps that were part of the original research environment.
EOF

cmake -S "$SCRIPT_DIR" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD_DIR" -j

echo "pdpcFeature build complete."

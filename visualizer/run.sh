#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

if [ ! -f "visualizer" ]; then
    echo "Building visualizer..."
    make
fi

echo "Starting RealSense Dual Visualizer (D435i + T265)..."
./visualizer

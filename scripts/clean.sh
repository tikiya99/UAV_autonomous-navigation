#!/bin/bash
# Clean build script for UAV Navigation Workspace

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR/../uav_navigation_ws"

echo "============================================"
echo "  Cleaning UAV Navigation Workspace"
echo "============================================"

cd "$WS_DIR"

echo "Removing build/, install/, log/ directories..."
rm -rf build/ install/ log/

echo ""
echo "Clean complete! Run ./scripts/build.sh to rebuild."
echo ""

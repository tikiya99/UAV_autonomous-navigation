#!/bin/bash
# Build script for UAV Navigation Workspace

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR/../uav_navigation_ws"

echo "============================================"
echo "  UAV Navigation Workspace Build Script"
echo "============================================"

cd "$WS_DIR"

# Source ROS2
source /opt/ros/humble/setup.bash

# Build options
BUILD_TYPE=${1:-"Release"}
PACKAGES=${2:-""}

if [ -n "$PACKAGES" ]; then
    echo "Building packages: $PACKAGES"
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=$BUILD_TYPE --packages-select $PACKAGES
else
    echo "Building all packages..."
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=$BUILD_TYPE
fi

echo ""
echo "Build complete! Source the workspace with:"
echo "  source $WS_DIR/install/setup.bash"
echo ""

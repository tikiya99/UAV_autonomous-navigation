#!/bin/bash
# Source script for UAV Navigation Workspace

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR/../uav_navigation_ws"

# Source ROS2
source /opt/ros/humble/setup.bash

# Source workspace
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
    echo "Sourced UAV Navigation workspace"
else
    echo "Workspace not built yet. Run ./scripts/build.sh first"
fi

# Export useful aliases
alias cb='cd $WS_DIR && colcon build --symlink-install'
alias cs='source $WS_DIR/install/setup.bash'
alias cw='cd $WS_DIR'

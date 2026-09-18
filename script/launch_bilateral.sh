#!/bin/bash
#
# Copyright 2025 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# ========= Configuration =========
ARM_SIDE=${1:-right_arm} # Required: left_arm or right_arm
LEADER_CAN_IF=$2         # Optional: leader CAN interface
FOLLOWER_CAN_IF=$3       # Optional: follower CAN interface
TMPDIR="/tmp/openarm_urdf_gen"

WS_DIR=${OPENARM_WS:-/home/ligx/workspace/openarm}
PKG_DIR="$WS_DIR/src/openarm_teleop"
XACRO_PATH="$WS_DIR/src/openarm_description/urdf/robot/oy.urdf.xacro"
BIN_PATH="$WS_DIR/build/openarm_teleop/bilateral_control"

# Validate arm side
if [[ "$ARM_SIDE" != "right_arm" && "$ARM_SIDE" != "left_arm" ]]; then
    echo "[ERROR] Invalid arm_side: $ARM_SIDE"
    echo "Usage: $0 <arm_side: right_arm|left_arm> [leader_can_if] [follower_can_if]"
    exit 1
fi

# Set default CAN interfaces if not provided (topology from init_can.sh:
# can0=follower right, can1=follower left, can2=leader right, can3=leader left)
if [ -z "$LEADER_CAN_IF" ]; then
    if [ "$ARM_SIDE" = "right_arm" ]; then
        LEADER_CAN_IF="can2"
    else
        LEADER_CAN_IF="can3"
    fi
fi

if [ -z "$FOLLOWER_CAN_IF" ]; then
    if [ "$ARM_SIDE" = "right_arm" ]; then
        FOLLOWER_CAN_IF="can0"
    else
        FOLLOWER_CAN_IF="can1"
    fi
fi

# File paths
LEADER_URDF_PATH="$TMPDIR/oy_leader.urdf"
FOLLOWER_URDF_PATH="$TMPDIR/oy_follower.urdf"

# Check xacro and binary
if [ ! -f "$XACRO_PATH" ]; then
    echo "[ERROR] Could not find xacro: $XACRO_PATH" >&2
    exit 1
fi

if [ ! -f "$BIN_PATH" ]; then
    echo "[ERROR] Compiled binary not found at: $BIN_PATH" >&2
    echo "Build first: cd $WS_DIR && colcon build --packages-select openarm_teleop" >&2
    exit 1
fi

# Source ROS 2 and the workspace (xacro, openarm_description, OpenArmCAN cmake)
# shellcheck source=/dev/null
source /opt/ros/humble/setup.bash
# shellcheck source=/dev/null
source "$WS_DIR/install/setup.bash"

# CAN bus is exclusive: teleop must not run together with ros2_control
if pgrep -f ros2_control_node >/dev/null 2>&1; then
    echo "[WARN] ros2_control_node is still running and will fight for the CAN bus." >&2
    echo "       Stop the bringup stack first." >&2
fi
echo "[INFO] Make sure the CAN interfaces are configured: sudo bash $WS_DIR/init_can.sh"

# Generate URDFs (oy.urdf.xacro defaults: arm_type:=oy ee_type:=openarm_hand)
echo "[INFO] Generating URDFs using xacro..."
mkdir -p "$TMPDIR"
if ! xacro "$XACRO_PATH" bimanual:=true -o "$LEADER_URDF_PATH"; then
    echo "[ERROR] Failed to generate URDFs."
    exit 1
fi
cp "$LEADER_URDF_PATH" "$FOLLOWER_URDF_PATH"

# Run binary from the package root (YamlLoader loads relative config/*.yaml)
echo "[INFO] Launching bilateral control..."
cd "$PKG_DIR"
"$BIN_PATH" "$LEADER_URDF_PATH" "$FOLLOWER_URDF_PATH" "$ARM_SIDE" "$LEADER_CAN_IF" "$FOLLOWER_CAN_IF"

# Cleanup
echo "[INFO] Cleaning up temporary files..."
rm -rf "$TMPDIR"

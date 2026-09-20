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
ARM_SIDE=${1:-right_arm}   # Required: left_arm or right_arm
LEADER_CAN_IF=$2           # Optional: leader CAN interface
FOLLOWER_CAN_IF=$3         # Optional: follower CAN interface
LEADER_GRAVITY_SCALE=$4    # Optional: leader gravity-compensation scale (overrides yaml)
FOLLOWER_GRAVITY_SCALE=$5  # Optional: follower gravity-compensation scale (requires $4)
# Validate arm side
if [[ "$ARM_SIDE" != "right_arm" && "$ARM_SIDE" != "left_arm" ]]; then
    echo "[ERROR] Invalid arm_side: $ARM_SIDE"
    echo "Usage: $0 <arm_side: right_arm|left_arm> [leader_can_if] [follower_can_if] [leader_gravity_scale] [follower_gravity_scale]"
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

# Load the system ROS environment and the nearest install tree. local_setup
# avoids replaying build-machine workspace paths embedded in setup.bash.
if ! command -v ros2 >/dev/null 2>&1; then
    ROS_SETUP="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
    if [ -f "$ROS_SETUP" ]; then
        source "$ROS_SETUP" || exit 1
    fi
fi

INSTALL_SETUP=""
if [ -n "${OPENARM_WS:-}" ]; then
    INSTALL_SETUP="$OPENARM_WS/install/local_setup.bash"
    if [ ! -f "$INSTALL_SETUP" ]; then
        echo "[ERROR] Install environment not found: $INSTALL_SETUP" >&2
        exit 1
    fi
else
    SEARCH_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd) || exit 1
    while [ "$SEARCH_DIR" != / ]; do
        if [ -f "$SEARCH_DIR/local_setup.bash" ]; then
            INSTALL_SETUP="$SEARCH_DIR/local_setup.bash"
            break
        elif [ -f "$SEARCH_DIR/install/local_setup.bash" ]; then
            INSTALL_SETUP="$SEARCH_DIR/install/local_setup.bash"
            break
        fi
        SEARCH_DIR=$(dirname -- "$SEARCH_DIR")
    done
fi
if [ -n "$INSTALL_SETUP" ]; then
    source "$INSTALL_SETUP" || exit 1
fi
if ! command -v ros2 >/dev/null 2>&1; then
    echo "[ERROR] ROS 2 is unavailable. Source your ROS and install environments first." >&2
    exit 1
fi

# Resolve all runtime resources from the active ROS package index.
DESCRIPTION_DIR=$(ros2 pkg prefix --share openarm_description) || exit 1
PKG_DIR=$(ros2 pkg prefix --share openarm_teleop) || exit 1
PKG_PREFIX=$(ros2 pkg prefix openarm_teleop) || exit 1
XACRO_PATH="$DESCRIPTION_DIR/urdf/robot/oy.urdf.xacro"
BIN_PATH="$PKG_PREFIX/lib/openarm_teleop/bilateral_control"
if [ ! -f "$XACRO_PATH" ]; then
    echo "[ERROR] Could not find installed xacro: $XACRO_PATH" >&2
    exit 1
fi
if [ ! -x "$BIN_PATH" ]; then
    echo "[ERROR] Installed executable not found: $BIN_PATH" >&2
    echo "Rebuild and install openarm_teleop: colcon build --packages-select openarm_teleop" >&2
    exit 1
fi
for CONFIG in leader follower; do
    if [ ! -f "$PKG_DIR/config/$CONFIG.yaml" ]; then
        echo "[ERROR] Installed configuration not found: $PKG_DIR/config/$CONFIG.yaml" >&2
        exit 1
    fi
done

URDF_DIR=$(mktemp -d "${TMPDIR:-/tmp}/openarm_urdf_gen.XXXXXX") || exit 1
trap 'rm -rf -- "$URDF_DIR"' EXIT
LEADER_URDF_PATH="$URDF_DIR/oy_leader.urdf"
FOLLOWER_URDF_PATH="$URDF_DIR/oy_follower.urdf"

# CAN bus is exclusive: teleop must not run together with ros2_control
if pgrep -f ros2_control_node >/dev/null 2>&1; then
    echo "[WARN] ros2_control_node is still running and will fight for the CAN bus." >&2
    echo "       Stop the bringup stack first." >&2
fi
echo "[INFO] Make sure the CAN interfaces are configured: $LEADER_CAN_IF and $FOLLOWER_CAN_IF"

# Generate URDFs (oy.urdf.xacro defaults: arm_type:=oy ee_type:=openarm_hand)
echo "[INFO] Generating URDFs using xacro..."
if ! xacro "$XACRO_PATH" bimanual:=true -o "$LEADER_URDF_PATH"; then
    echo "[ERROR] Failed to generate URDFs."
    exit 1
fi
cp "$LEADER_URDF_PATH" "$FOLLOWER_URDF_PATH" || exit 1

# Run binary from the package root (YamlLoader loads relative config/*.yaml)
echo "[INFO] Launching bilateral control..."
cd "$PKG_DIR" || exit 1

# Pass gravity scale overrides when provided (follower scale requires the
# leader scale due to positional argument order).
if [ -n "$FOLLOWER_GRAVITY_SCALE" ] && [ -z "$LEADER_GRAVITY_SCALE" ]; then
    echo "[ERROR] follower_gravity_scale (arg 5) requires leader_gravity_scale (arg 4)." >&2
    exit 1
fi
GRAVITY_SCALE_ARGS=()
if [ -n "$LEADER_GRAVITY_SCALE" ]; then
    GRAVITY_SCALE_ARGS+=("$LEADER_GRAVITY_SCALE")
fi
if [ -n "$FOLLOWER_GRAVITY_SCALE" ]; then
    GRAVITY_SCALE_ARGS+=("$FOLLOWER_GRAVITY_SCALE")
fi

"$BIN_PATH" "$LEADER_URDF_PATH" "$FOLLOWER_URDF_PATH" "$ARM_SIDE" "$LEADER_CAN_IF" "$FOLLOWER_CAN_IF" "${GRAVITY_SCALE_ARGS[@]}"

exit $?

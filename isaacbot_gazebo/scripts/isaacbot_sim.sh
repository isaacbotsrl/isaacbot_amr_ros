#!/usr/bin/env bash

# Copyright 2025 Mechatronics Academy
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

# Runs the Ardennes Gazebo simulation from a fresh terminal, whatever the login shell set up
# for the real isaacbot:
#   - ISAACBOT_AMR_ROS_BUILD_TYPE=simulation (~/.bashrc exports "hardware").
#   - rmw_zenoh on a local router bound to localhost. ZENOH_CONFIG_OVERRIDE from the login
#     shell (client mode to the isaacbot's router) is dropped, so the sim never joins the isaacbot.
#     FastDDS is not used: inter-process discovery does not work on this WSL host.
#
# Usage:
#   isaacbot_sim.sh [--build] [launch args...]   start the simulation (Ctrl+C stops everything)
#   source isaacbot_sim.sh                       only set up this shell, e.g. a second terminal
#                                             for rover_navigation or ros2 CLI tools
#
# Examples:
#   isaacbot_sim.sh --build
#   ISAACBOT_SYSTEM_USE_GPS=true isaacbot_sim.sh use_rviz:=False gz_headless_mode:=True
#   source isaacbot_sim.sh && ros2 launch rover_navigation bringup.launch.py \
#     use_sim_time:=True localization_source:=slam
#
# Environment (all optional):
#   ISAACBOT_WS                  workspace root (default: found from this script, else
#                             ~/ros2_ws/rover_a1)
#   ISAACBOT_SYSTEM_NAMESPACE    robot namespace (default: isaacbot, as in docker-compose)
#   ISAACBOT_SYSTEM_USE_GPS, ISAACBOT_SYSTEM_GPS_MAP_TF
#                             passed through to the simulation

_isaacbot_sim_is_sourced() { [ "${BASH_SOURCE[0]}" != "$0" ]; }

_isaacbot_sim_find_ws() {
    if [ -n "${ISAACBOT_WS:-}" ]; then
        echo "$ISAACBOT_WS"
        return
    fi
    # With --symlink-install the installed script links back here, so walk up from the source.
    local dir
    dir=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")
    while [ "$dir" != "/" ]; do
        if [ -d "$dir/src" ] && [ -d "$dir/src/isaacbot_amr_ros" ]; then
            echo "$dir"
            return
        fi
        dir=$(dirname "$dir")
    done
    echo "$HOME/ros2_ws/rover_a1"
}

_isaacbot_sim_setup_env() {
    ISAACBOT_WS=$(_isaacbot_sim_find_ws)
    export ISAACBOT_WS

    export ISAACBOT_AMR_ROS_BUILD_TYPE=simulation
    export ISAACBOT_SYSTEM_NAMESPACE="${ISAACBOT_SYSTEM_NAMESPACE:-isaacbot}"

    unset ZENOH_CONFIG_OVERRIDE ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp

    # A stale overlay (e.g. a hardware install sourced by ~/.bashrc) would shadow this build.
    unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH

    local distro
    distro=$(ls /opt/ros | head -n 1)
    # ROS setup scripts are not nounset-safe.
    # shellcheck disable=SC1090
    source "/opt/ros/${distro}/setup.bash"

    if [ -f "$ISAACBOT_WS/install/setup.bash" ]; then
        # shellcheck disable=SC1091
        source "$ISAACBOT_WS/install/setup.bash"
    fi

    # A ros2 CLI daemon started from the login shell keeps the isaacbot's zenoh config, and the
    # CLI would then list the isaacbot's graph instead of the sim's. It restarts on demand.
    ros2 daemon stop > /dev/null 2>&1 || true
}

_isaacbot_sim_router_running() {
    ss -Hltn 'sport = :7447' 2>/dev/null | grep -q LISTEN
}

if _isaacbot_sim_is_sourced; then
    _isaacbot_sim_setup_env
    if ! _isaacbot_sim_router_running; then
        echo "isaacbot_sim: no zenoh router on :7447 yet - start the simulation with isaacbot_sim.sh first." >&2
    fi
    echo "isaacbot_sim: shell ready (RMW=$RMW_IMPLEMENTATION, namespace=$ISAACBOT_SYSTEM_NAMESPACE, ws=$ISAACBOT_WS)."
    return 0
fi

set -eo pipefail

BUILD=false
LAUNCH_ARGS=()
for arg in "$@"; do
    case "$arg" in
        --build) BUILD=true ;;
        -h | --help)
            sed -n '17,38p' "$(readlink -f "$0")" | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        *) LAUNCH_ARGS+=("$arg") ;;
    esac
done

_isaacbot_sim_setup_env

missing=()
for pkg in gz_ros2_control ros_gz_bridge ros_gz_sim; do
    ros2 pkg prefix "$pkg" > /dev/null 2>&1 || missing+=("ros-${ROS_DISTRO}-${pkg//_/-}")
done
if [ ${#missing[@]} -gt 0 ]; then
    echo "isaacbot_sim: missing packages. Install them with:" >&2
    echo "  sudo apt install -y ${missing[*]}" >&2
    exit 1
fi

if [ "$BUILD" = true ]; then
    (
        cd "$ISAACBOT_WS"
        colcon build --symlink-install --packages-up-to isaacbot_metapackage rover_autonomy \
            --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
    )
    # shellcheck disable=SC1091
    source "$ISAACBOT_WS/install/setup.bash"
fi

if ! ros2 pkg prefix isaacbot_gazebo > /dev/null 2>&1; then
    echo "isaacbot_sim: isaacbot_gazebo is not built in $ISAACBOT_WS - run: isaacbot_sim.sh --build" >&2
    exit 1
fi

ROUTER_PID=""
cleanup() {
    if [ -n "$ROUTER_PID" ]; then
        kill "$ROUTER_PID" 2> /dev/null || true
        wait "$ROUTER_PID" 2> /dev/null || true
    fi
}
trap cleanup EXIT

if _isaacbot_sim_router_running; then
    echo "isaacbot_sim: reusing the zenoh router already listening on :7447."
else
    # Localhost only; multicast scouting is off in the default router config, so this router
    # cannot find or be found by the isaacbot's.
    ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/127.0.0.1:7447"]' \
        ros2 run rmw_zenoh_cpp rmw_zenohd > "/tmp/isaacbot_sim_zenohd_$$.log" 2>&1 &
    ROUTER_PID=$!
    for _ in $(seq 1 50); do
        _isaacbot_sim_router_running && break
        if ! kill -0 "$ROUTER_PID" 2> /dev/null; then
            echo "isaacbot_sim: zenoh router failed, see /tmp/isaacbot_sim_zenohd_$$.log" >&2
            exit 1
        fi
        sleep 0.1
    done
    echo "isaacbot_sim: zenoh router started (pid $ROUTER_PID, localhost:7447)."
fi

echo "isaacbot_sim: namespace=$ISAACBOT_SYSTEM_NAMESPACE use_gps=${ISAACBOT_SYSTEM_USE_GPS:-false} ws=$ISAACBOT_WS"
ros2 launch isaacbot_gazebo simulation.launch.py "${LAUNCH_ARGS[@]}"

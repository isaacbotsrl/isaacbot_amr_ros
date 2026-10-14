#!/usr/bin/env bash
# Copyright 2026 IsaacBot
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
#
# Writes (or refreshes, or removes) the "isaacbot-pc setup" block in ~/.bashrc: ROS 2, the ardennes
# workspace overlay, ISAACBOT_AMR_* variables, and the Zenoh settings that put this PC on the isaacbot's
# ROS graph. Idempotent: the block between the markers is replaced in place, never duplicated,
# and the rest of the file is left byte for byte. A backup is written before any change.
#
#   isaacbot_amr_scripts/setup_isaacbot_amr_pc.sh                      # default: isaacbot at 192.168.1.201
#   isaacbot_amr_scripts/setup_isaacbot_amr_pc.sh --isaacbot-ip 10.0.0.5 --dry-run
#   isaacbot_amr_scripts/setup_isaacbot_amr_pc.sh --remove
set -euo pipefail

START_MARKER="# >>> isaacbot-pc setup >>>"
END_MARKER="# <<< isaacbot-pc setup <<<"

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# <ws>/src/isaacbot_amr_ros/isaacbot_amr_scripts -> <ws>
DEFAULT_WS=$(cd "$SCRIPT_DIR/../../.." && pwd)

default_distro() {
    local distros=()
    [ -d /opt/ros ] && mapfile -t distros < <(ls /opt/ros 2>/dev/null)
    if [ -d /opt/ros/lyrical ]; then echo lyrical
    elif [ "${#distros[@]}" -eq 1 ]; then echo "${distros[0]}"
    else echo lyrical
    fi
}

WORKSPACE=$DEFAULT_WS
ISAACBOT_AMR_IP=192.168.1.201
NAMESPACE=isaacbot
DISTRO=$(default_distro)
BUILD_TYPE=hardware
DOMAIN_ID=0
BASHRC="$HOME/.bashrc"
DRY_RUN=false
REMOVE=false

usage() {
    cat <<EOF
Usage: $(basename "$0") [options]

Writes the "isaacbot-pc setup" block into ~/.bashrc (replacing an existing one in place).

  --workspace PATH     ardennes colcon workspace       (default: $DEFAULT_WS)
  --isaacbot-ip IP        isaacbot LAN address (Zenoh router) (default: $ISAACBOT_AMR_IP)
  --namespace NAME     ISAACBOT_AMR_SYSTEM_NAMESPACE           (default: $NAMESPACE)
  --distro NAME        ROS 2 distro under /opt/ros      (default: $DISTRO)
  --build-type TYPE    hardware | simulation            (default: $BUILD_TYPE)
  --domain-id N        ROS_DOMAIN_ID                    (default: $DOMAIN_ID)
  --bashrc FILE        file to edit                     (default: ~/.bashrc)
  --dry-run            print the block and the diff, change nothing
  --remove             delete the block
  -h, --help           this help

The PC joins the isaacbot's graph as a direct Zenoh client of the router in ardennes-zenoh-router
(ZENOH_CONFIG_OVERRIDE). isaacbot_amr_gazebo/scripts/isaacbot_amr_sim.sh clears that for the local simulation.
EOF
}

die() { echo "error: $*" >&2; exit 1; }

while [ $# -gt 0 ]; do
    case "$1" in
        --workspace) WORKSPACE=${2:?}; shift 2 ;;
        --isaacbot-ip) ISAACBOT_AMR_IP=${2:?}; shift 2 ;;
        --namespace) NAMESPACE=${2:?}; shift 2 ;;
        --distro) DISTRO=${2:?}; shift 2 ;;
        --build-type) BUILD_TYPE=${2:?}; shift 2 ;;
        --domain-id) DOMAIN_ID=${2:?}; shift 2 ;;
        --bashrc) BASHRC=${2:?}; shift 2 ;;
        --dry-run) DRY_RUN=true; shift ;;
        --remove) REMOVE=true; shift ;;
        -h|--help) usage; exit 0 ;;
        *) usage >&2; die "unknown option: $1" ;;
    esac
done

# --- validation ------------------------------------------------------------------------------
if [[ ! $ISAACBOT_AMR_IP =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]]; then die "--isaacbot-ip '$ISAACBOT_AMR_IP' is not an IPv4 address"; fi
IFS=. read -r -a octets <<< "$ISAACBOT_AMR_IP"
for o in "${octets[@]}"; do [ "$o" -le 255 ] || die "--isaacbot-ip '$ISAACBOT_AMR_IP' is not an IPv4 address"; done
[[ $NAMESPACE =~ ^[A-Za-z_][A-Za-z0-9_]*$ ]] || die "--namespace '$NAMESPACE' must match [A-Za-z_][A-Za-z0-9_]*"
[[ $DISTRO =~ ^[a-z]+$ ]] || die "--distro '$DISTRO' is not a ROS distro name"
case "$BUILD_TYPE" in hardware|simulation) ;; *) die "--build-type must be hardware or simulation" ;; esac
[[ $DOMAIN_ID =~ ^[0-9]+$ ]] && [ "$DOMAIN_ID" -le 232 ] || die "--domain-id must be 0..232"
[[ $WORKSPACE = /* ]] || die "--workspace must be an absolute path"
[ -f "/opt/ros/$DISTRO/setup.bash" ] || echo "warning: /opt/ros/$DISTRO/setup.bash not found - install ROS 2 $DISTRO first" >&2
[ -d "$WORKSPACE/src" ] || echo "warning: $WORKSPACE/src not found - is --workspace right?" >&2

# --- the block -------------------------------------------------------------------------------
render_block() {
    cat <<EOF
$START_MARKER
# Managed by isaacbot_amr_ros/isaacbot_amr_scripts/setup_isaacbot_amr_pc.sh - edits inside this block are overwritten.
source /opt/ros/$DISTRO/setup.bash
[ -f $WORKSPACE/install/setup.bash ] && source $WORKSPACE/install/setup.bash
export ISAACBOT_AMR_ROS_BUILD_TYPE=$BUILD_TYPE
export ISAACBOT_AMR_SYSTEM_NAMESPACE=$NAMESPACE

# Ardennes ROS 2 graph: Zenoh RMW, domain $DOMAIN_ID (the isaacbot uses the default, 0).
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=$DOMAIN_ID

# ROS_LOCALHOST_ONLY (deprecated) and ROS_AUTOMATIC_DISCOVERY_RANGE are FastDDS-era leftovers
# that confine nodes to a private graph - clear them. Then join the isaacbot's graph directly:
# every node is a Zenoh client of the router in ardennes-zenoh-router. No local router is used.
# isaacbot_amr_gazebo/scripts/isaacbot_amr_sim.sh clears this override for the local simulation (it runs its
# own router on 127.0.0.1:7447); unset it too for off-isaacbot tests.
unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/$ISAACBOT_AMR_IP:7447"]'
EOF
    # Literal \$HOME / \$PATH: expanded when .bashrc runs. The case guard keeps nested shells
    # from stacking duplicate PATH entries.
    cat <<'EOF'
case ":$PATH:" in *":$HOME/.local/bin:"*) ;; *) export PATH="$HOME/.local/bin:$PATH" ;; esac
case ":$PATH:" in *":$HOME/balena/bin:"*) ;; *) export PATH="$HOME/balena/bin:$PATH" ;; esac
export QT_QPA_PLATFORM=xcb
EOF
    echo "$END_MARKER"
}

# --- edit ------------------------------------------------------------------------------------
[ -e "$BASHRC" ] || { $DRY_RUN || touch "$BASHRC"; }
starts=0; ends=0
if [ -f "$BASHRC" ]; then
    starts=$(grep -cxF "$START_MARKER" "$BASHRC" || true)
    ends=$(grep -cxF "$END_MARKER" "$BASHRC" || true)
fi
if [ "$starts" -gt 1 ] || [ "$ends" -gt 1 ] || [ "$starts" -ne "$ends" ]; then
    die "$BASHRC has $starts start and $ends end markers - fix it by hand ('$START_MARKER' ... '$END_MARKER')"
fi
if [ "$starts" -eq 1 ]; then
    s=$(grep -nxF "$START_MARKER" "$BASHRC" | cut -d: -f1)
    e=$(grep -nxF "$END_MARKER" "$BASHRC" | cut -d: -f1)
    [ "$s" -lt "$e" ] || die "$BASHRC: end marker comes before the start marker - fix it by hand"
fi

tmp=$(mktemp)
block=$(mktemp)
trap 'rm -f "$tmp" "$block"' EXIT
render_block > "$block"

if [ "$starts" -eq 1 ]; then
    # Replace in place (or drop, with --remove); everything outside the markers is copied as is.
    awk -v s="$START_MARKER" -v e="$END_MARKER" -v blockfile="$block" -v remove="$REMOVE" '
        $0 == s { inside = 1; if (remove != "true") { while ((getline line < blockfile) > 0) print line }; next }
        $0 == e { inside = 0; next }
        !inside { print }
    ' "$BASHRC" > "$tmp"
elif $REMOVE; then
    echo "No isaacbot-pc setup block in $BASHRC - nothing to remove."
    exit 0
else
    cat "$BASHRC" > "$tmp" 2>/dev/null || true
    # Keep a blank line between existing content and the block.
    if [ -s "$tmp" ] && [ -n "$(tail -c1 "$tmp")" ]; then echo >> "$tmp"; fi
    [ -s "$tmp" ] && echo >> "$tmp"
    cat "$block" >> "$tmp"
fi

if $DRY_RUN; then
    $REMOVE || { echo "--- block ---"; cat "$block"; }
    echo "--- diff ($BASHRC) ---"
    diff -u "$BASHRC" "$tmp" 2>/dev/null || true
    echo "(dry run: nothing written)"
    exit 0
fi

if [ -f "$BASHRC" ] && cmp -s "$BASHRC" "$tmp"; then
    echo "$BASHRC is already up to date - unchanged."
    exit 0
fi

backup="$BASHRC.bak-$(date +%Y%m%d-%H%M%S)"
[ -s "$BASHRC" ] && cp -p "$BASHRC" "$backup" && echo "Backup: $backup"
cat "$tmp" > "$BASHRC"   # keep the file's inode, owner and mode
if $REMOVE; then
    echo "Removed the isaacbot-pc setup block from $BASHRC."
else
    echo "Wrote the isaacbot-pc setup block to $BASHRC (isaacbot $ISAACBOT_AMR_IP, namespace $NAMESPACE, $BUILD_TYPE, ROS $DISTRO)."
fi
echo "Open a new terminal, or run: source $BASHRC"

#!/usr/bin/env bash
# Replay a recorded bag with sim_time and optionally launch the planning/control stack.
set -euo pipefail

BAG_PATH="${1:?Usage: replay_bag.sh <bag_directory> [--no-stack]}"
LAUNCH_STACK=true

for arg in "$@"; do
  [[ "$arg" == "--no-stack" ]] && LAUNCH_STACK=false
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
source "$REPO_ROOT/install/setup.bash"

if $LAUNCH_STACK; then
  echo "=== Launching planning/control stack (use_sim_time=true) ==="
  ros2 launch av_bringup planning_control.launch.py use_sim_time:=true &
  STACK_PID=$!
  sleep 2
fi

echo "=== Replaying bag: $BAG_PATH ==="
ros2 bag play "$BAG_PATH" \
  --clock \
  --rate 1.0 \
  --remap /tf:=/tf_replay

if $LAUNCH_STACK; then
  wait $STACK_PID
fi

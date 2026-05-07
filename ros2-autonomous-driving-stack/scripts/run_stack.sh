#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

source "$REPO_ROOT/install/setup.bash"

ros2 launch av_bringup full_stack.launch.py \
  use_sim_time:=false \
  log_level:=info \
  "$@"

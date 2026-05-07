#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$REPO_ROOT"

echo "=== Installing dependencies ==="
rosdep update --rosdistro humble
rosdep install --from-paths src --ignore-src -r -y

echo "=== Building ==="
colcon build \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --event-handlers console_cohesion+

echo "=== Build complete ==="
echo "Run: source install/setup.bash"

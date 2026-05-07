#!/usr/bin/env bash
# Print live topic Hz and latency for the AV stack pipeline.
set -euo pipefail

TOPICS=(
  /sensing/lidar_points
  /sensing/imu
  /localization/pose
  /localization/odom
  /perception/tracked_objects
  /planning/trajectory
  /control/cmd
  /vehicle/command
)

echo "Checking topic Hz (Ctrl-C to stop) ..."
for topic in "${TOPICS[@]}"; do
  echo "--- $topic ---"
  timeout 3 ros2 topic hz "$topic" 2>/dev/null || echo "  (no messages in 3 s)"
done

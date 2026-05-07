#!/usr/bin/env bash
# Record all AV stack topics for 30 seconds into a timestamped bag directory.
set -euo pipefail

OUTPUT_DIR="${1:-/tmp/av_bag_$(date +%Y%m%d_%H%M%S)}"
DURATION="${2:-30}"

echo "Recording to: $OUTPUT_DIR"
echo "Duration:     ${DURATION}s"

ros2 bag record \
  --output "$OUTPUT_DIR" \
  --duration "$DURATION" \
  /sensing/lidar_points \
  /sensing/camera_image \
  /sensing/imu \
  /sensing/gnss \
  /localization/pose \
  /localization/odom \
  /perception/objects \
  /perception/tracked_objects \
  /planning/global_path \
  /planning/behavior_state \
  /planning/trajectory \
  /control/steering \
  /control/cmd \
  /vehicle/command \
  /tf \
  /tf_static

echo "Bag saved to: $OUTPUT_DIR"

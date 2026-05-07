# System Architecture

## Design Philosophy

Each package has a single responsibility and communicates exclusively through typed ROS2 topics and
services. No package imports another package's source — only `av_msgs` is shared. This mirrors the
Autoware modular architecture and allows any subsystem to be swapped independently.

## Data Flow

```
Sensors (LiDAR, Camera, IMU, GNSS)
        │
        ▼
  av_sensing          — voxel filtering, image rectification, time-sync
        │
   /sensing/*
        ├──────────────────────────────────────────┐
        ▼                                          ▼
  av_localization                           av_perception
  (EKF: GNSS + IMU)                    (detection + tracking)
        │                                          │
  /localization/pose              /perception/tracked_objects
        └──────────────────┬───────────────────────┘
                           ▼
                     av_planning
              (global → behavior → trajectory)
                           │
                   /planning/trajectory
                           │
                           ▼
                     av_control
              (Pure Pursuit + PID speed)
                           │
                     /control/cmd
                           │
                           ▼
                av_vehicle_interface
                (CAN gateway + adapter)
                           │
                   /vehicle/command
                           │
                           ▼
                    Physical Vehicle
```

## Package Responsibilities

### av_sensing
- Subscribes: raw sensor topics (driver level)
- Publishes: `/sensing/lidar_points`, `/sensing/camera_image`, `/sensing/imu`, `/sensing/gnss`
- Processing: voxel-grid downsampling, camera undistortion, IMU bias removal, GNSS NMEA parsing
- Key design: time-stamped, frame-attached messages; no filtering logic (kept in perception)

### av_localization
- Subscribes: `/sensing/imu`, `/sensing/gnss`
- Publishes: `/localization/pose`, `/localization/odom`, TF `map → odom → base_link`
- Algorithm: Extended Kalman Filter with 6-DOF state `[x, y, z, roll, pitch, yaw]`
- Key design: publishes TF so all downstream nodes operate in `map` frame without extra transforms

### av_perception
- Subscribes: `/sensing/lidar_points`, `/localization/pose`
- Publishes: `/perception/objects`, `/perception/tracked_objects`
- Algorithm: clustering-based 3D object detection + SORT multi-object tracker
- Key design: track IDs are stable across frames, enabling prediction in the planner

### av_planning
- Subscribes: `/perception/tracked_objects`, `/localization/pose`
- Publishes: `/planning/trajectory`
- Layers:
  1. **Global planner** — Dijkstra on a lane graph, produces coarse path
  2. **Behavior planner** — FSM (LANE_FOLLOWING → STOPPING → EMERGENCY_STOP)
  3. **Trajectory generator** — quintic polynomial spline with velocity profile
- Key design: behavior state is logged for post-hoc analysis

### av_control
- Subscribes: `/planning/trajectory`, `/localization/odom`
- Publishes: `/control/cmd`
- Algorithms:
  - Lateral: Pure Pursuit (open-loop preview) with Stanley fallback at low speed
  - Longitudinal: PID with acceleration feed-forward
- Key design: tracking error is published as a diagnostic topic

### av_vehicle_interface
- Subscribes: `/control/cmd`
- Publishes: `/vehicle/command`, CAN frames
- Key design: decouples control algorithm from vehicle-specific CAN encoding; easy to swap for
  different platforms

## Failure Modes and Mitigations

See `failure-cases.md` for the full treatment. Short summary:

| Failure | Detection | Response |
|---|---|---|
| LiDAR dropout | `/sensing/lidar_points` age > 200 ms | EMERGENCY_STOP via behavior FSM |
| GNSS jump | Mahalanobis distance > threshold in EKF | Reject GNSS update, keep IMU propagation |
| Perception latency spike | `/perception/tracked_objects` age > 300 ms | Hold last known objects for 1 s |
| Planning timeout | `/planning/trajectory` age > 500 ms | Comfort deceleration to stop |
| CAN bus error | ACK timeout on vehicle interface | Publish EMERGENCY_STOP command |

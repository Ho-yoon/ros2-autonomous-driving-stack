# Topic Interface Design

## Principles

1. **Typed, versioned messages** — all cross-package data uses `av_msgs`; no `std_msgs/String` hacks.
2. **Stamped + framed** — every spatial message carries `std_msgs/Header` with `stamp` and `frame_id`.
3. **Covariance included** — pose and odometry carry covariance matrices so downstream EKF/filters
   can consume uncertainty correctly.
4. **Arrays over repeated publishes** — `DetectedObject[]` in one message rather than one message
   per object, keeping latency deterministic.

## Custom Message Definitions

### `av_msgs/DetectedObject`
```
std_msgs/Header header
uint32 id
string label                  # "car", "pedestrian", "cyclist", …
float32 confidence
geometry_msgs/PoseWithCovariance pose
geometry_msgs/Vector3 dimensions  # bounding box [l, w, h] in metres
geometry_msgs/Twist velocity
```

### `av_msgs/TrackedObject`
```
av_msgs/DetectedObject detection
uint32 track_id
uint32 age                    # frames since first detection
float32 existence_probability
geometry_msgs/Twist predicted_velocity
```

### `av_msgs/TrajectoryPoint`
```
std_msgs/Header header
geometry_msgs/Pose pose
float32 longitudinal_velocity_mps
float32 lateral_velocity_mps
float32 heading_rate_rps
float32 acceleration_mps2
float32 front_wheel_angle_rad
float64 time_from_start       # seconds from trajectory start
```

### `av_msgs/VehicleState`
```
std_msgs/Header header
float32 steering_tire_angle   # rad
float32 longitudinal_velocity # m/s
float32 lateral_velocity      # m/s
float32 gear_shift            # 0=P, 1=R, 2=N, 3=D
float32 hand_brake
bool emergency_stop
```

## Standard Message Usage

| Topic | std type | Notes |
|---|---|---|
| `/sensing/lidar_points` | `sensor_msgs/PointCloud2` | fields: x,y,z,intensity,ring |
| `/sensing/camera_image` | `sensor_msgs/Image` | encoding: bgr8 |
| `/sensing/imu` | `sensor_msgs/Imu` | frame_id: imu_link |
| `/sensing/gnss` | `sensor_msgs/NavSatFix` | WGS84 |
| `/localization/pose` | `geometry_msgs/PoseWithCovarianceStamped` | frame_id: map |
| `/localization/odom` | `nav_msgs/Odometry` | child_frame_id: base_link |
| `/control/cmd` | `geometry_msgs/Twist` | linear.x = speed, angular.z = yaw_rate |

## Service Definitions

### `av_msgs/ResetScenario`
```
# Request
string scenario_id
---
# Response
bool success
string message
```

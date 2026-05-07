# TF2 Frame Tree

## Frame Hierarchy

```
map  (ENU, fixed world frame)
 │
 └── odom  (published by av_localization/ekf_localizer)
      │     drifts slowly; continuous, no jumps
      │
      └── base_link  (vehicle centre, published by av_localization/ekf_localizer)
           │          rigid body; all sensor frames are children
           │
           ├── lidar_link   (static, published by robot_state_publisher)
           │    offset: [x=1.5, y=0.0, z=1.8] from base_link
           │
           ├── camera_link  (static)
           │    offset: [x=1.8, y=0.0, z=1.4] from base_link
           │
           ├── imu_link     (static)
           │    offset: [x=0.0, y=0.0, z=0.5] from base_link
           │
           └── gnss_link    (static)
                offset: [x=0.0, y=0.0, z=1.9] from base_link
```

## Frame Semantics

| Frame | Publisher | Type | Notes |
|---|---|---|---|
| `map` | — | Origin | ENU coordinate system; set at first GNSS fix |
| `odom` | `ekf_localizer_node` | Dynamic | Integrated from IMU; monotonically drifts |
| `base_link` | `ekf_localizer_node` | Dynamic | Corrected by GNSS; may jump on correction |
| `lidar_link` | `robot_state_publisher` | Static | Calibrated extrinsic; loaded from URDF |
| `camera_link` | `robot_state_publisher` | Static | Calibrated extrinsic |
| `imu_link` | `robot_state_publisher` | Static | Coincides with physical IMU chip centre |
| `gnss_link` | `robot_state_publisher` | Static | Coincides with GNSS antenna phase centre |

## Why Two Odom → Base Link Transforms?

The `map → odom` transform absorbs the global correction from GNSS so that `odom → base_link`
remains smooth (no discontinuities for the control loop). The control loop subscribes to
`/localization/odom` (child: `base_link`, parent: `odom`) for jitter-free feedback. Global
planning uses the `map` frame pose.

## Usage in Nodes

```cpp
// Look up base_link position in map frame
auto tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

// Transform a LiDAR point into base_link frame
geometry_msgs::msg::PointStamped pt_lidar, pt_base;
pt_lidar.header.frame_id = "lidar_link";
tf2::doTransform(pt_lidar, pt_base, tf_buffer_->lookupTransform("base_link", "lidar_link", ...));
```

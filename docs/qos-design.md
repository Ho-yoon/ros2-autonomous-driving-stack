# QoS Design

## Policy Matrix

| Topic | Reliability | Durability | History | Depth | Rationale |
|---|---|---|---|---|---|
| `/sensing/lidar_points` | BEST_EFFORT | VOLATILE | KEEP_LAST | 5 | High-rate; dropping old scans acceptable |
| `/sensing/camera_image` | BEST_EFFORT | VOLATILE | KEEP_LAST | 3 | Bandwidth-limited; late frames useless |
| `/sensing/imu` | RELIABLE | VOLATILE | KEEP_LAST | 10 | Critical for EKF; must not drop |
| `/sensing/gnss` | RELIABLE | VOLATILE | KEEP_LAST | 5 | Low-rate, important |
| `/localization/pose` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST | 1 | Late-joining nodes need last known pose |
| `/localization/odom` | RELIABLE | VOLATILE | KEEP_LAST | 10 | Control loop input |
| `/perception/objects` | RELIABLE | VOLATILE | KEEP_LAST | 10 | Must not lose detections |
| `/perception/tracked_objects` | RELIABLE | VOLATILE | KEEP_LAST | 10 | Planner input |
| `/planning/trajectory` | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST | 1 | Control needs last trajectory on (re)start |
| `/control/cmd` | RELIABLE | VOLATILE | KEEP_LAST | 5 | Safety-critical |
| `/vehicle/command` | RELIABLE | VOLATILE | KEEP_LAST | 1 | Only latest command is meaningful |

## Notes

### TRANSIENT_LOCAL Durability
Topics carrying state that late-joining nodes need (pose, trajectory) use `TRANSIENT_LOCAL` so a
node that restarts does not have to wait for the next publication cycle.

### Compatibility Caveat
A publisher with BEST_EFFORT and a subscriber with RELIABLE are incompatible under DDS — the
subscription will not receive messages. All sensing topics are BEST_EFFORT on both sides.
Downstream packages that require guaranteed delivery subscribe to `/localization` or
`/perception` topics instead.

### Implementation in config
QoS profiles are declared in `src/av_bringup/config/qos.yaml` and loaded by each node via
`rclcpp::QoSInitialization` from a shared helper in `av_bringup`. This avoids duplicating QoS
constants across packages.

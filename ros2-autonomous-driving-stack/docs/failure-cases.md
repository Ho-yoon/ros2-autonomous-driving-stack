# Failure Cases and Mitigations

## Sensor Failures

### LiDAR Dropout
- **Detection**: `object_detector_node` monitors `/sensing/lidar_points` stamp; triggers if age > 200 ms
- **Impact**: Perception produces no new detections
- **Response**: Behavior FSM transitions `LANE_FOLLOWING → EMERGENCY_STOP`; vehicle decelerates
- **Recovery**: Automatic on lidar restoration; FSM requires explicit reset via `/scenario/reset` service

### GNSS Jump
- **Detection**: EKF Mahalanobis distance check; `gnss_imu_fusion` rejects measurements outside 3-sigma gate
- **Impact**: Localization falls back to IMU-only dead reckoning; uncertainty grows
- **Response**: Covariance in `/localization/pose` inflates; planner extends safety margins
- **Recovery**: GNSS accepted again after 3 consecutive consistent fixes

### Camera Failure
- **Detection**: `camera_preprocessor_node` publishes a `/sensing/camera_status` diagnostic
- **Impact**: Perception loses camera-based features (traffic light detection falls back to map priors)
- **Response**: Warning logged; stack continues with LiDAR-only perception

### IMU Bias Drift
- **Detection**: EKF innovation sequence monitored; sustained bias detected after ~5 min stationary
- **Impact**: Yaw estimate drifts; heading error grows
- **Response**: Planner reduces speed; operator notified via `/diagnostics`

## Software Failures

### Perception Latency Spike
- **Detection**: `behavior_planner_node` monitors `/perception/tracked_objects` stamp
- **Threshold**: Age > 300 ms
- **Response**: Hold last known object set for up to 1 s, then expand safety footprint
- **Recovery**: Automatic when latency recovers

### Planning Timeout
- **Detection**: `pure_pursuit_controller` monitors `/planning/trajectory` stamp
- **Threshold**: Age > 500 ms
- **Response**: Comfort deceleration (−2 m/s²) to standstill; EMERGENCY_STOP after 3 s
- **Recovery**: Automatic on trajectory republication

### Node Crash
- **Detection**: `av_bringup` uses `on_exit` respawn events for critical nodes
- **Nodes with respawn**: `ekf_localizer`, `behavior_planner`, `pure_pursuit_controller`
- **Response**: Node restarts; subscribes to TRANSIENT_LOCAL topics to recover last state
- **Caveat**: CAN gateway requires manual restart to re-open the socket

## Vehicle Interface Failures

### CAN Bus Error
- **Detection**: `can_gateway_node` ACK timeout on every frame send
- **Response**: Publishes `/vehicle/command` with `emergency_stop=true` flag; logs CAN errno
- **Recovery**: Requires manual operator intervention

### Actuator Saturation
- **Detection**: Commanded steering angle or throttle exceeds vehicle limits defined in `vehicle.yaml`
- **Response**: Clamp with warning log; no emergency stop (soft limit)
- **Recovery**: Planner re-plans within limits

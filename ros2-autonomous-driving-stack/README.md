# ROS2 Autonomous Driving Stack

A full-stack autonomous driving system implemented in ROS2 Humble, demonstrating production-level
architecture from sensor input through vehicle command output.

## What This Project Proves

| Capability | Evidence |
|---|---|
| ROS2 package architecture | 8 composable packages with clean interface boundaries |
| Sensing → Localization → Perception → Planning → Control pipeline | Full data-flow from raw sensor to actuator command |
| TF2 coordinate frame management | `base_link`, `map`, `odom`, `lidar`, `camera` frame tree |
| QoS configuration | Per-topic RELIABILITY/DURABILITY/HISTORY tuning in `qos.yaml` |
| Custom message/service design | `av_msgs` package with typed interfaces for every pipeline stage |
| rosbag-based replay & regression testing | `record_bag.sh` + `replay_bag.sh` + `test_topic_contract.py` |
| Dockerized reproducible environment | Single `docker compose up` brings up the full stack |
| C++/Python mixed development | Core nodes in C++17, launch/test/scripts in Python3 |

---

## Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│                        ROS2 Autonomous Driving Stack                  │
│                                                                        │
│  ┌──────────────┐   /sensing/lidar_points    ┌──────────────────────┐ │
│  │  av_sensing  │──────────────────────────▶│  av_localization     │ │
│  │              │   /sensing/camera_image    │  (EKF: GNSS+IMU)     │ │
│  │  LiDAR pre-  │──────────────────────────▶│                      │ │
│  │  Camera pre- │   /sensing/imu             │  /localization/pose  │ │
│  │  processor   │──────────────────────────▶│  /localization/odom  │ │
│  └──────────────┘   /sensing/gnss            └──────────┬───────────┘ │
│                                                          │             │
│  ┌──────────────┐   /perception/objects                  │             │
│  │ av_perception│◀─────────────────────────────────────┘             │
│  │              │──────────────────────────▶┌──────────────────────┐ │
│  │  Detector    │   /perception/tracked_obj │   av_planning        │ │
│  │  Tracker     │                           │                      │ │
│  └──────────────┘                           │  Global planner      │ │
│                                             │  Behavior planner    │ │
│                                             │  Trajectory gen      │ │
│                                             │  /planning/trajectory│ │
│                                             └──────────┬───────────┘ │
│                                                          │             │
│  ┌──────────────┐   /vehicle/command                    │             │
│  │av_vehicle_   │◀─────────────────────────────────────┘             │
│  │interface     │◀──────── /control/cmd ───┌──────────────────────┐ │
│  │              │                           │   av_control         │ │
│  │CAN gateway   │                           │  Pure Pursuit        │ │
│  └──────────────┘                           │  Stanley             │ │
│                                             │  PID speed           │ │
│                                             └──────────────────────┘ │
└──────────────────────────────────────────────────────────────────────┘
```

### TF Frame Tree

```
map
 └── odom
      └── base_link
           ├── lidar_link
           ├── camera_link
           ├── imu_link
           └── gnss_link
```

---

## Quick Start

### With Docker (recommended)

```bash
git clone https://github.com/Ho-yoon/ros2-autonomous-driving-stack.git
cd ros2-autonomous-driving-stack
docker compose up
```

### Native ROS2 Humble

```bash
# Prerequisites: ROS2 Humble, colcon, rosdep
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch av_bringup full_stack.launch.py
```

### Replay a rosbag

```bash
./scripts/record_bag.sh          # record 30 s of topics
./scripts/replay_bag.sh          # replay and visualise in RViz
```

---

## Package Overview

| Package | Language | Role |
|---|---|---|
| `av_msgs` | — | Shared message/service definitions |
| `av_bringup` | Python | Launch files and system-wide configuration |
| `av_sensing` | C++ | LiDAR/camera preprocessing, IMU/GNSS passthrough |
| `av_localization` | C++ | EKF-based GNSS+IMU fusion, odometry |
| `av_perception` | C++ | Object detection and multi-object tracking |
| `av_planning` | C++ | Global path, behavior FSM, trajectory generation |
| `av_control` | C++ | Pure Pursuit / Stanley lateral + PID longitudinal |
| `av_vehicle_interface` | C++ | CAN-bus gateway and vehicle command adaptation |

---

## Key Topics

| Topic | Type | QoS | Description |
|---|---|---|---|
| `/sensing/lidar_points` | `sensor_msgs/PointCloud2` | BEST_EFFORT / VOLATILE | Raw LiDAR scan |
| `/sensing/camera_image` | `sensor_msgs/Image` | BEST_EFFORT / VOLATILE | Raw camera frame |
| `/sensing/imu` | `sensor_msgs/Imu` | RELIABLE / VOLATILE | IMU at 100 Hz |
| `/sensing/gnss` | `sensor_msgs/NavSatFix` | RELIABLE / VOLATILE | GNSS fix |
| `/localization/pose` | `geometry_msgs/PoseWithCovarianceStamped` | RELIABLE / TRANSIENT_LOCAL | EKF pose estimate |
| `/localization/odom` | `nav_msgs/Odometry` | RELIABLE / VOLATILE | Filtered odometry |
| `/perception/objects` | `av_msgs/DetectedObject[]` | RELIABLE / VOLATILE | Raw detections |
| `/perception/tracked_objects` | `av_msgs/TrackedObject[]` | RELIABLE / VOLATILE | Tracked with IDs |
| `/planning/trajectory` | `av_msgs/TrajectoryPoint[]` | RELIABLE / TRANSIENT_LOCAL | Reference trajectory |
| `/control/cmd` | `geometry_msgs/Twist` | RELIABLE / VOLATILE | Velocity command |
| `/vehicle/command` | `av_msgs/VehicleState` | RELIABLE / VOLATILE | Final CAN command |

---

## Metrics

- **Planning latency**: measured end-to-end in `test_launch_stack.py`
- **Lateral tracking error**: RMS logged by `av_control` node
- **Object detection**: precision/recall evaluated against rosbag ground truth
- **rosbag replay reproducibility**: deterministic topic contract verified by `test_topic_contract.py`

---

## Demo

> RViz screenshot and CARLA/Gazebo simulation video will be added.

Replay a recorded scenario without a simulator:

```bash
./scripts/replay_bag.sh data/scenario_01.bag
python3 scripts/plot_trajectory.py
```

---

## CI

GitHub Actions runs on every push:
1. `colcon build` inside ROS2 Humble Docker image
2. Unit tests (`test_planner.cpp`, `test_controller.cpp`)
3. Integration tests (`test_launch_stack.py`, `test_topic_contract.py`)

See `.github/workflows/ci.yml`.

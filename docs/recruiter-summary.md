# What This Repository Demonstrates (Recruiter Summary)

This document maps repository artifacts to the competencies expected of a mid-to-senior autonomous
driving software engineer.

## ROS2 Expertise

| Evidence | Location |
|---|---|
| 8 composable ROS2 packages with correct `package.xml` / `CMakeLists.txt` | `src/av_*/` |
| Custom message and service definitions | `src/av_msgs/msg/`, `src/av_msgs/srv/` |
| Component-based node architecture (zero-copy intra-process) | all `src/av_*/src/*.cpp` |
| QoS design with documented rationale | `src/av_bringup/config/qos.yaml`, `docs/qos-design.md` |
| TF2 frame tree with static and dynamic transforms | `docs/tf-tree.md` |
| Multi-file launch system with arguments and event handlers | `src/av_bringup/launch/` |
| rosdep / colcon build workflow | `scripts/build.sh` |

## Autonomous Driving Domain Knowledge

| Evidence | Location |
|---|---|
| Sensing pipeline (LiDAR voxel filtering, camera rectification) | `src/av_sensing/src/` |
| EKF-based GNSS+IMU fusion | `src/av_localization/src/ekf_localizer.cpp` |
| 3D object detection + SORT tracker | `src/av_perception/src/` |
| Global planner (Dijkstra on lane graph) | `src/av_planning/src/global_planner.cpp` |
| Behavior FSM (LANE_FOLLOWING / STOPPING / EMERGENCY_STOP) | `src/av_planning/src/behavior_planner.cpp` |
| Trajectory generation (quintic polynomial) | `src/av_planning/src/trajectory_generator.cpp` |
| Pure Pursuit + Stanley lateral control | `src/av_control/src/pure_pursuit_controller.cpp` |
| PID longitudinal speed control | `src/av_control/src/pid_speed_controller.cpp` |
| CAN gateway abstraction | `src/av_vehicle_interface/src/can_gateway_node.cpp` |

## Software Engineering Practices

| Evidence | Location |
|---|---|
| Hermetic Docker environment | `docker/Dockerfile`, `docker/docker-compose.yml` |
| CI pipeline (build + test on every push) | `.github/workflows/ci.yml` |
| Unit tests (gtest) | `tests/unit/` |
| Integration tests (rosbag replay + topic contract) | `tests/integration/` |
| rosbag record / replay workflow | `scripts/record_bag.sh`, `scripts/replay_bag.sh` |
| Trajectory visualisation script | `scripts/plot_trajectory.py` |
| Structured failure-mode documentation | `docs/failure-cases.md` |
| Interface-first design (topic contracts documented before implementation) | `docs/topic-interface.md` |

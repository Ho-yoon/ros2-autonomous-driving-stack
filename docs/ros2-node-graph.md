# ROS2 Node Graph

## Nodes

| Node | Package | Executable | Spin type |
|---|---|---|---|
| `lidar_preprocessor` | av_sensing | `lidar_preprocessor_node` | Component |
| `camera_preprocessor` | av_sensing | `camera_preprocessor_node` | Component |
| `ekf_localizer` | av_localization | `ekf_localizer_node` | Component |
| `gnss_imu_fusion` | av_localization | `gnss_imu_fusion_node` | Component |
| `object_detector` | av_perception | `object_detector_node` | Component |
| `tracking` | av_perception | `tracking_node` | Component |
| `global_planner` | av_planning | `global_planner_node` | Component |
| `behavior_planner` | av_planning | `behavior_planner_node` | Component |
| `trajectory_generator` | av_planning | `trajectory_generator_node` | Component |
| `pure_pursuit_controller` | av_control | `pure_pursuit_node` | Component |
| `pid_speed_controller` | av_control | `pid_speed_node` | Component |
| `vehicle_command_adapter` | av_vehicle_interface | `vehicle_command_adapter_node` | Component |
| `can_gateway` | av_vehicle_interface | `can_gateway_node` | Component |

All nodes are registered as `rclcpp_components` and loaded into a `ComponentManager` at runtime,
allowing zero-copy intra-process communication when collocated in the same process.

## Topic Graph (abbreviated)

```
lidar_driver ──/sensing/lidar_raw──▶ lidar_preprocessor
                                           │
                                     /sensing/lidar_points
                                           ├──────────────▶ object_detector
                                           └──────────────▶ (rosbag record)

camera_driver ──/sensing/camera_raw──▶ camera_preprocessor
                                             │
                                       /sensing/camera_image
                                             └──────────────▶ object_detector

imu_driver   ──/sensing/imu_raw──▶ (passthrough) ──/sensing/imu──▶ gnss_imu_fusion
gnss_driver  ──/sensing/gnss_raw─▶ (passthrough) ──/sensing/gnss─▶ gnss_imu_fusion

gnss_imu_fusion ──/localization/pose──▶ ekf_localizer
                                              │
                                        /localization/odom
                                              ├──▶ behavior_planner
                                              └──▶ pure_pursuit_controller
                                              └──▶ pid_speed_controller

object_detector ──/perception/objects──▶ tracking_node
tracking_node   ──/perception/tracked_objects──▶ behavior_planner

behavior_planner ──/planning/behavior_state──▶ trajectory_generator
global_planner   ──/planning/global_path──▶    trajectory_generator
trajectory_generator ──/planning/trajectory──▶ pure_pursuit_controller
                                             └▶ pid_speed_controller

pure_pursuit_controller ──/control/steering──▶ vehicle_command_adapter
pid_speed_controller    ──/control/speed────▶ vehicle_command_adapter
vehicle_command_adapter ──/vehicle/command──▶ can_gateway_node
```

## Services

| Service | Server Node | Client Node | Purpose |
|---|---|---|---|
| `/scenario/reset` | behavior_planner | test harness | Reset FSM for new test run |
| `/map/load` | global_planner | av_bringup | Load lane graph at startup |

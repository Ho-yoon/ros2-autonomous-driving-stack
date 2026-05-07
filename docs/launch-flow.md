# Launch Flow

## Entry Points

| Launch file | Purpose |
|---|---|
| `full_stack.launch.py` | Bring up all 8 packages |
| `perception_only.launch.py` | Sensing + Perception only (for detector development) |
| `planning_control.launch.py` | Localization + Planning + Control (replay from rosbag) |

## full_stack.launch.py — Startup Sequence

```
1. robot_state_publisher      ← publishes static TF from URDF
2. av_sensing                 ← starts LiDAR and camera preprocessing
3. av_localization            ← starts EKF (waits for /sensing/imu ready)
4. av_perception              ← starts detector + tracker
5. av_planning                ← loads map, starts planners
6. av_control                 ← starts controller nodes
7. av_vehicle_interface       ← opens CAN gateway last
```

Steps 2–7 are launched with `OnProcessStart` event handlers in sequence to avoid publishing to
topics before subscribers exist (which wastes messages even with TRANSIENT_LOCAL).

## Launch Arguments

```bash
ros2 launch av_bringup full_stack.launch.py \
    vehicle_config:=config/vehicle.yaml \
    use_sim_time:=true \
    log_level:=info \
    record_bag:=false
```

| Argument | Default | Description |
|---|---|---|
| `vehicle_config` | `config/vehicle.yaml` | Vehicle geometry and CAN mapping |
| `use_sim_time` | `false` | Set `true` when replaying a rosbag or using Gazebo |
| `log_level` | `info` | ROS2 log level for all nodes |
| `record_bag` | `false` | Auto-record all topics on launch |

## Config Loading

Each node loads its YAML config via `declare_parameter` / `get_parameter` pattern:

```python
# in launch file
Node(
    package='av_control',
    executable='pure_pursuit_node',
    parameters=[
        PathJoinSubstitution([FindPackageShare('av_control'), 'config', 'control.yaml']),
        {'use_sim_time': use_sim_time},
    ],
)
```

Nodes use `rcl_interfaces/ParameterEvent` to support runtime reconfiguration (e.g., tuning PID
gains without restarting).

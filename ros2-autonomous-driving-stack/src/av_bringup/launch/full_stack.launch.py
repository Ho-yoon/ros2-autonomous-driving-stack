"""Launch the full autonomous driving stack."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vehicle_config = LaunchConfiguration("vehicle_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")
    record_bag = LaunchConfiguration("record_bag")

    bringup_share = FindPackageShare("av_bringup")
    qos_config = PathJoinSubstitution([bringup_share, "config", "qos.yaml"])

    declare_args = [
        DeclareLaunchArgument(
            "vehicle_config",
            default_value=PathJoinSubstitution([bringup_share, "config", "vehicle.yaml"]),
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("record_bag", default_value="false"),
    ]

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
    )

    sensing = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("av_sensing"), "launch", "sensing.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "qos_config": qos_config,
        }.items(),
    )

    localization = TimerAction(
        period=1.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("av_localization"), "launch", "localization.launch.py"]
                    )
                ),
                launch_arguments={"use_sim_time": use_sim_time, "qos_config": qos_config}.items(),
            )
        ],
    )

    perception = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("av_perception"), "launch", "perception.launch.py"]
                    )
                ),
                launch_arguments={"use_sim_time": use_sim_time, "qos_config": qos_config}.items(),
            )
        ],
    )

    planning = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("av_planning"), "launch", "planning.launch.py"]
                    )
                ),
                launch_arguments={"use_sim_time": use_sim_time, "qos_config": qos_config}.items(),
            )
        ],
    )

    control = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("av_control"), "launch", "control.launch.py"]
                    )
                ),
                launch_arguments={"use_sim_time": use_sim_time, "qos_config": qos_config}.items(),
            )
        ],
    )

    vehicle_interface = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("av_vehicle_interface"),
                            "launch",
                            "vehicle_interface.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "vehicle_config": vehicle_config,
                }.items(),
            )
        ],
    )

    rosbag_recorder = Node(
        package="ros2bag",
        executable="record",
        arguments=["-a", "-o", "/tmp/av_stack_recording"],
        condition=IfCondition(record_bag),
    )

    return LaunchDescription(
        declare_args
        + [
            robot_state_publisher,
            sensing,
            localization,
            perception,
            planning,
            control,
            vehicle_interface,
            rosbag_recorder,
        ]
    )

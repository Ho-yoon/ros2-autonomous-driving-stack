"""Launch localization + planning + control for rosbag replay."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    bringup_share = FindPackageShare("av_bringup")
    qos_config = PathJoinSubstitution([bringup_share, "config", "qos.yaml"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("av_localization"), "launch", "localization.launch.py"]
                    )
                ),
                launch_arguments={"use_sim_time": use_sim_time, "qos_config": qos_config}.items(),
            ),
            TimerAction(
                period=1.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [FindPackageShare("av_planning"), "launch", "planning.launch.py"]
                            )
                        ),
                        launch_arguments={
                            "use_sim_time": use_sim_time,
                            "qos_config": qos_config,
                        }.items(),
                    )
                ],
            ),
            TimerAction(
                period=2.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [FindPackageShare("av_control"), "launch", "control.launch.py"]
                            )
                        ),
                        launch_arguments={
                            "use_sim_time": use_sim_time,
                            "qos_config": qos_config,
                        }.items(),
                    )
                ],
            ),
        ]
    )

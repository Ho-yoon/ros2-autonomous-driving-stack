"""Launch sensing + perception only (for detector development and tuning)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    bringup_share = FindPackageShare("av_bringup")
    qos_config = PathJoinSubstitution([bringup_share, "config", "qos.yaml"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("av_sensing"), "launch", "sensing.launch.py"]
                    )
                ),
                launch_arguments={"use_sim_time": use_sim_time, "qos_config": qos_config}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("av_perception"), "launch", "perception.launch.py"]
                    )
                ),
                launch_arguments={"use_sim_time": use_sim_time, "qos_config": qos_config}.items(),
            ),
        ]
    )

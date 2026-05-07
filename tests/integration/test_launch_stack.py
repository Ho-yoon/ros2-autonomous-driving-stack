"""Integration test: launch the full stack and verify each pipeline stage publishes."""

import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.node import Node

from av_msgs.msg import TrajectoryPointArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


@pytest.mark.rostest
def generate_test_description():
    planning_control_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            launch.substitutions.PathJoinSubstitution([
                launch.substitutions.FindPackageShare("av_bringup"),
                "launch",
                "planning_control.launch.py",
            ])
        ),
        launch_arguments={"use_sim_time": "false"}.items(),
    )

    return launch.LaunchDescription([
        planning_control_launch,
        launch_testing.actions.ReadyToTest(),
    ])


class TestStackPublishes(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_stack_publishes")

    def tearDown(self):
        self.node.destroy_node()

    def _wait_for_message(self, topic, msg_type, timeout=10.0):
        received = []

        def cb(msg):
            received.append(msg)

        sub = self.node.create_subscription(msg_type, topic, cb, 10)
        start = time.time()
        while not received and time.time() - start < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_subscription(sub)
        return received

    def test_localization_pose_published(self):
        msgs = self._wait_for_message(
            "/localization/pose", PoseWithCovarianceStamped
        )
        self.assertGreater(len(msgs), 0, "/localization/pose received no messages")

    def test_localization_odom_published(self):
        msgs = self._wait_for_message("/localization/odom", Odometry)
        self.assertGreater(len(msgs), 0, "/localization/odom received no messages")

    def test_planning_trajectory_published(self):
        msgs = self._wait_for_message("/planning/trajectory", TrajectoryPointArray)
        self.assertGreater(len(msgs), 0, "/planning/trajectory received no messages")
        self.assertGreater(len(msgs[0].points), 0, "Trajectory is empty")


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)

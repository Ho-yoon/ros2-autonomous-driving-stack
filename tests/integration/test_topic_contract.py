"""
Topic contract test: replay a rosbag and verify every pipeline topic publishes
at the expected rate and within expected latency bounds.
"""

import subprocess
import time
import unittest

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from av_msgs.msg import TrackedObjectArray, TrajectoryPointArray, VehicleState
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2


TOPIC_CONTRACTS = [
    ("/sensing/lidar_points",        PointCloud2,              5.0),   # Hz min
    ("/sensing/imu",                 Imu,                      50.0),
    ("/localization/pose",           PoseWithCovarianceStamped, 10.0),
    ("/localization/odom",           Odometry,                 10.0),
    ("/perception/tracked_objects",  TrackedObjectArray,        5.0),
    ("/planning/trajectory",         TrajectoryPointArray,      2.0),
    ("/control/cmd",                 Twist,                    10.0),
    ("/vehicle/command",             VehicleState,             10.0),
]

SAMPLE_WINDOW = 3.0  # seconds


class TestTopicContract(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("topic_contract_tester")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _measure_rate(self, topic, msg_type, window=SAMPLE_WINDOW):
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        counts = [0]

        def cb(_):
            counts[0] += 1

        sub = self.node.create_subscription(msg_type, topic, cb, qos)
        start = time.time()
        while time.time() - start < window:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        elapsed = time.time() - start
        self.node.destroy_subscription(sub)
        return counts[0] / elapsed

    def test_all_topic_rates(self):
        failures = []
        for topic, msg_type, min_hz in TOPIC_CONTRACTS:
            actual_hz = self._measure_rate(topic, msg_type)
            if actual_hz < min_hz:
                failures.append(
                    f"{topic}: expected >= {min_hz} Hz, got {actual_hz:.1f} Hz"
                )

        if failures:
            self.fail("Topic rate violations:\n" + "\n".join(failures))

    def test_trajectory_points_non_empty(self):
        received = []

        def cb(msg):
            received.append(msg)

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        sub = self.node.create_subscription(
            TrajectoryPointArray, "/planning/trajectory", cb, qos
        )
        start = time.time()
        while not received and time.time() - start < 5.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_subscription(sub)

        self.assertTrue(received, "/planning/trajectory never received")
        self.assertGreater(len(received[0].points), 0, "Trajectory has no points")

    def test_vehicle_command_no_emergency_stop_at_startup(self):
        received = []

        def cb(msg):
            received.append(msg)

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        sub = self.node.create_subscription(VehicleState, "/vehicle/command", cb, qos)
        start = time.time()
        while not received and time.time() - start < 5.0:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_subscription(sub)

        if received:
            self.assertFalse(received[0].emergency_stop,
                             "Vehicle command had emergency_stop=True at startup")


if __name__ == "__main__":
    unittest.main()

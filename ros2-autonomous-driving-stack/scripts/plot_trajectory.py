#!/usr/bin/env python3
"""
Plot planned trajectory and actual driven path from a rosbag.

Usage:
    python3 scripts/plot_trajectory.py <bag_directory>
"""

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def read_bag_trajectory(bag_path: str):
    """Read /planning/trajectory and /localization/odom from a rosbag2 SQLite bag."""
    try:
        from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError:
        print("rosbag2_py not available — using demo data")
        return None, None

    storage_options = StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = ConverterOptions("", "")
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    traj_xy, odom_xy = [], []

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == "/planning/trajectory":
            msg_type = get_message("av_msgs/msg/TrajectoryPointArray")
            msg = deserialize_message(data, msg_type)
            for pt in msg.points:
                traj_xy.append((pt.pose.position.x, pt.pose.position.y))
        elif topic == "/localization/odom":
            msg_type = get_message("nav_msgs/msg/Odometry")
            msg = deserialize_message(data, msg_type)
            odom_xy.append((
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
            ))

    return np.array(traj_xy) if traj_xy else None, np.array(odom_xy) if odom_xy else None


def demo_data():
    """Generate demo data if no bag is available."""
    t = np.linspace(0, 100, 200)
    traj = np.column_stack([t, np.zeros_like(t)])
    noise = np.random.normal(0, 0.15, (200, 2))
    odom = traj + noise
    return traj, odom


def main():
    bag_path = sys.argv[1] if len(sys.argv) > 1 else None

    if bag_path:
        traj, odom = read_bag_trajectory(bag_path)
    else:
        traj, odom = None, None

    if traj is None or odom is None:
        print("Using demo data (run with a bag path for real data)")
        traj, odom = demo_data()

    # Compute lateral error
    if len(traj) > 0 and len(odom) > 0:
        n = min(len(traj), len(odom))
        diff = odom[:n] - traj[:n]
        lateral_error = np.linalg.norm(diff, axis=1)
        rms_error = np.sqrt(np.mean(lateral_error ** 2))
        max_error = np.max(lateral_error)
        print(f"Lateral tracking error — RMS: {rms_error:.3f} m  Max: {max_error:.3f} m")

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    ax = axes[0]
    ax.plot(traj[:, 0], traj[:, 1], "b--", linewidth=1.5, label="Planned trajectory")
    ax.plot(odom[:, 0], odom[:, 1], "r-", linewidth=1.0, label="Actual path (odom)")
    ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]")
    ax.set_title("Path tracking"); ax.legend(); ax.set_aspect("equal"); ax.grid(True)

    ax2 = axes[1]
    if len(traj) > 0 and len(odom) > 0:
        n = min(len(traj), len(odom))
        ax2.plot(lateral_error, "g-", linewidth=1.0)
        ax2.axhline(rms_error, color="orange", linestyle="--", label=f"RMS={rms_error:.3f} m")
        ax2.set_xlabel("Sample"); ax2.set_ylabel("Lateral error [m]")
        ax2.set_title("Lateral tracking error"); ax2.legend(); ax2.grid(True)

    plt.tight_layout()
    out = Path("trajectory_plot.png")
    plt.savefig(out, dpi=150)
    print(f"Plot saved to {out.resolve()}")
    plt.show()


if __name__ == "__main__":
    main()

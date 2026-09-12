#!/usr/bin/env python3
from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from visualization_msgs.msg import Marker


def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class PumaRvizTargetPublisher(Node):
    def __init__(self) -> None:
        super().__init__("puma_rviz_target_publisher")

        self.declare_parameter("frame_id", "world")
        self.declare_parameter("pose_topic", "/puma/target_pose")
        self.declare_parameter("marker_topic", "/visualization_marker")
        self.declare_parameter("status_topic", "/puma/moveit_status")

        self.frame_id = self.get_parameter("frame_id").value
        self.pose_topic = self.get_parameter("pose_topic").value
        self.marker_topic = self.get_parameter("marker_topic").value
        self.status_topic = self.get_parameter("status_topic").value

        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.joint_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.on_joint_states,
            10,
        )

        self.target_x = 0.45
        self.target_y = 0.0
        self.target_z = 0.35
        self.roll_deg = 180.0
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0

        self.last_joint_state: Optional[JointState] = None

        self.publish_status(
            "Publisher ready. Commands: pose x y z roll pitch yaw | xyz x y z | rpy r p y | publish | status | demo | quit"
        )

    def publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def on_joint_states(self, msg: JointState) -> None:
        self.last_joint_state = msg

    def build_pose(self) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.frame_id

        pose.pose.position.x = float(self.target_x)
        pose.pose.position.y = float(self.target_y)
        pose.pose.position.z = float(self.target_z)

        roll = math.radians(self.roll_deg)
        pitch = math.radians(self.pitch_deg)
        yaw = math.radians(self.yaw_deg)

        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        return pose

    def publish_marker(self, pose: PoseStamped) -> None:
        marker = Marker()
        marker.header = pose.header
        marker.ns = "puma_manual_target"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.scale.z = 0.04
        marker.color.a = 1.0
        marker.color.r = 0.1
        marker.color.g = 0.9
        marker.color.b = 0.2
        self.marker_pub.publish(marker)

    def publish_target(self) -> None:
        pose = self.build_pose()
        self.pose_pub.publish(pose)
        self.publish_marker(pose)

        self.publish_status(
            f"Published target pose: "
            f"x={self.target_x:.3f}, y={self.target_y:.3f}, z={self.target_z:.3f}, "
            f"roll={self.roll_deg:.1f}, pitch={self.pitch_deg:.1f}, yaw={self.yaw_deg:.1f}"
        )

    def print_status(self) -> None:
        self.publish_status(
            f"Current target: "
            f"x={self.target_x:.3f}, y={self.target_y:.3f}, z={self.target_z:.3f}, "
            f"roll={self.roll_deg:.1f}, pitch={self.pitch_deg:.1f}, yaw={self.yaw_deg:.1f}"
        )

        if self.last_joint_state is not None and self.last_joint_state.name:
            joint_text = ", ".join(
                f"{n}={p:.3f}" for n, p in zip(self.last_joint_state.name, self.last_joint_state.position)
            )
            self.publish_status(f"Last joint state: {joint_text}")

    def run_cli(self) -> None:
        while rclpy.ok():
            try:
                raw = input("puma_target> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break

            if not raw:
                continue

            parts = raw.split()
            cmd = parts[0].lower()

            try:
                if cmd == "pose" and len(parts) == 7:
                    self.target_x = float(parts[1])
                    self.target_y = float(parts[2])
                    self.target_z = float(parts[3])
                    self.roll_deg = float(parts[4])
                    self.pitch_deg = float(parts[5])
                    self.yaw_deg = float(parts[6])
                    self.print_status()

                elif cmd == "xyz" and len(parts) == 4:
                    self.target_x = float(parts[1])
                    self.target_y = float(parts[2])
                    self.target_z = float(parts[3])
                    self.print_status()

                elif cmd == "rpy" and len(parts) == 4:
                    self.roll_deg = float(parts[1])
                    self.pitch_deg = float(parts[2])
                    self.yaw_deg = float(parts[3])
                    self.print_status()

                elif cmd == "publish":
                    self.publish_target()

                elif cmd == "status":
                    self.print_status()

                elif cmd == "demo":
                    demo_targets = [
                        (0.45, 0.00, 0.35, 180.0, 0.0, 0.0),
                        (0.40, 0.10, 0.30, 180.0, 0.0, 30.0),
                        (0.35, -0.10, 0.40, 180.0, 0.0, -30.0),
                    ]
                    for x, y, z, r, p, yw in demo_targets:
                        self.target_x = x
                        self.target_y = y
                        self.target_z = z
                        self.roll_deg = r
                        self.pitch_deg = p
                        self.yaw_deg = yw
                        self.publish_target()
                        rclpy.spin_once(self, timeout_sec=0.1)

                elif cmd in {"quit", "exit"}:
                    break

                else:
                    self.publish_status(
                        "Unknown command. Use: pose x y z roll pitch yaw | xyz x y z | rpy r p y | publish | status | demo | quit"
                    )

            except ValueError:
                self.publish_status("Invalid numeric input")

            rclpy.spin_once(self, timeout_sec=0.01)


def main(args=None):
    rclpy.init(args=args)
    node = PumaRvizTargetPublisher()
    try:
        node.run_cli()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

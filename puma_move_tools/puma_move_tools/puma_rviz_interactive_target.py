#!/usr/bin/env python3
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import String
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker

from interactive_markers import InteractiveMarkerServer, MenuHandler
from visualization_msgs.msg import InteractiveMarkerFeedback


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class PumaRvizInteractiveTarget(Node):
    def __init__(self) -> None:
        super().__init__("puma_rviz_interactive_target")

        self.declare_parameter("frame_id", "world")
        self.declare_parameter("pose_topic", "/puma/target_pose")
        self.declare_parameter("marker_topic", "/visualization_marker")
        self.declare_parameter("status_topic", "/puma/moveit_status")
        self.declare_parameter("server_topic_ns", "/puma/interactive_marker")

        self.declare_parameter("default_x", 0.45)
        self.declare_parameter("default_y", 0.0)
        self.declare_parameter("default_z", 0.35)
        self.declare_parameter("default_roll_deg", 180.0)
        self.declare_parameter("default_pitch_deg", 0.0)
        self.declare_parameter("default_yaw_deg", 0.0)

        self.declare_parameter("auto_publish_on_pose_update", True)

        self.frame_id = self.get_parameter("frame_id").value
        self.pose_topic = self.get_parameter("pose_topic").value
        self.marker_topic = self.get_parameter("marker_topic").value
        self.status_topic = self.get_parameter("status_topic").value
        self.server_topic_ns = self.get_parameter("server_topic_ns").value

        self.default_x = float(self.get_parameter("default_x").value)
        self.default_y = float(self.get_parameter("default_y").value)
        self.default_z = float(self.get_parameter("default_z").value)
        self.default_roll_deg = float(self.get_parameter("default_roll_deg").value)
        self.default_pitch_deg = float(self.get_parameter("default_pitch_deg").value)
        self.default_yaw_deg = float(self.get_parameter("default_yaw_deg").value)

        self.auto_publish = bool(self.get_parameter("auto_publish_on_pose_update").value)

        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.server = InteractiveMarkerServer(self, self.server_topic_ns)
        self.menu_handler = MenuHandler()

        self.menu_publish = self.menu_handler.insert("Publish target pose", callback=self.on_menu_publish)
        self.menu_reset = self.menu_handler.insert("Reset to default", callback=self.on_menu_reset)
        self.menu_toggle_auto = self.menu_handler.insert("Toggle auto-publish", callback=self.on_menu_toggle_auto)

        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = self.frame_id
        self.current_pose.pose.position.x = self.default_x
        self.current_pose.pose.position.y = self.default_y
        self.current_pose.pose.position.z = self.default_z
        self.current_pose.pose.orientation = euler_to_quaternion(
            math.radians(self.default_roll_deg),
            math.radians(self.default_pitch_deg),
            math.radians(self.default_yaw_deg),
        )

        self.insert_interactive_marker()
        self.publish_status("Interactive RViz target is ready")

    def publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def publish_target_pose(self) -> None:
        self.current_pose.header.stamp = self.get_clock().now().to_msg()
        self.pose_pub.publish(self.current_pose)
        self.publish_visual_marker()
        self.publish_status(
            f"Published interactive target pose: "
            f"x={self.current_pose.pose.position.x:.3f}, "
            f"y={self.current_pose.pose.position.y:.3f}, "
            f"z={self.current_pose.pose.position.z:.3f}"
        )

    def publish_visual_marker(self) -> None:
        marker = Marker()
        marker.header = self.current_pose.header
        marker.ns = "puma_interactive_target"
        marker.id = 100
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = self.current_pose.pose
        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.scale.z = 0.04
        marker.color.a = 1.0
        marker.color.r = 0.2
        marker.color.g = 0.7
        marker.color.b = 1.0
        self.marker_pub.publish(marker)

    def make_box_marker(self) -> Marker:
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 0.2
        marker.color.g = 0.7
        marker.color.b = 1.0
        marker.color.a = 1.0
        return marker

    def add_control(self, int_marker: InteractiveMarker, name: str, orientation, interaction_mode: int) -> None:
        control = InteractiveMarkerControl()
        control.name = name
        control.orientation = orientation
        control.interaction_mode = interaction_mode
        control.always_visible = False
        int_marker.controls.append(control)

    def insert_interactive_marker(self) -> None:
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.frame_id
        int_marker.name = "puma_target_marker"
        int_marker.description = "PUMA Target"
        int_marker.scale = 0.20
        int_marker.pose = self.current_pose.pose

        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append(self.make_box_marker())
        int_marker.controls.append(box_control)

        self.add_control(
            int_marker,
            "move_x",
            Quaternion(x=1.0, y=0.0, z=0.0, w=1.0),
            InteractiveMarkerControl.MOVE_AXIS,
        )
        self.add_control(
            int_marker,
            "move_y",
            Quaternion(x=0.0, y=1.0, z=0.0, w=1.0),
            InteractiveMarkerControl.MOVE_AXIS,
        )
        self.add_control(
            int_marker,
            "move_z",
            Quaternion(x=0.0, y=0.0, z=1.0, w=1.0),
            InteractiveMarkerControl.MOVE_AXIS,
        )

        self.add_control(
            int_marker,
            "rotate_x",
            Quaternion(x=1.0, y=0.0, z=0.0, w=1.0),
            InteractiveMarkerControl.ROTATE_AXIS,
        )
        self.add_control(
            int_marker,
            "rotate_y",
            Quaternion(x=0.0, y=1.0, z=0.0, w=1.0),
            InteractiveMarkerControl.ROTATE_AXIS,
        )
        self.add_control(
            int_marker,
            "rotate_z",
            Quaternion(x=0.0, y=0.0, z=1.0, w=1.0),
            InteractiveMarkerControl.ROTATE_AXIS,
        )

        self.server.insert(int_marker, feedback_callback=self.process_feedback)
        self.menu_handler.apply(self.server, int_marker.name)
        self.server.applyChanges()
        self.publish_visual_marker()

    def process_feedback(self, feedback: InteractiveMarkerFeedback) -> None:
        self.current_pose.header.frame_id = self.frame_id
        self.current_pose.header.stamp = self.get_clock().now().to_msg()
        self.current_pose.pose = feedback.pose
        self.publish_visual_marker()

        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            if self.auto_publish:
                self.publish_target_pose()

    def on_menu_publish(self, feedback: InteractiveMarkerFeedback) -> None:
        self.current_pose.header.frame_id = self.frame_id
        self.current_pose.pose = feedback.pose
        self.publish_target_pose()

    def on_menu_reset(self, feedback: InteractiveMarkerFeedback) -> None:
        self.current_pose.pose.position.x = self.default_x
        self.current_pose.pose.position.y = self.default_y
        self.current_pose.pose.position.z = self.default_z
        self.current_pose.pose.orientation = euler_to_quaternion(
            math.radians(self.default_roll_deg),
            math.radians(self.default_pitch_deg),
            math.radians(self.default_yaw_deg),
        )

        self.server.clear()
        self.insert_interactive_marker()
        self.publish_target_pose()
        self.publish_status("Interactive target reset to default")

    def on_menu_toggle_auto(self, feedback: InteractiveMarkerFeedback) -> None:
        self.auto_publish = not self.auto_publish
        self.publish_status(f"Auto-publish is now {'ON' if self.auto_publish else 'OFF'}")


def main(args=None):
    rclpy.init(args=args)
    node = PumaRvizInteractiveTarget()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

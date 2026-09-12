#!/usr/bin/env python3
from __future__ import annotations

import yaml
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    DisplayTrajectory,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
)
from trajectory_msgs.msg import JointTrajectory


class PumaPoseToJointBridge(Node):
    """
    Bridge node:

    Input:
      /puma/target_pose                geometry_msgs/PoseStamped

    Output:
      /puma/final_joint_target         sensor_msgs/JointState
      /puma/moveit_status              std_msgs/String
      /visualization_marker            visualization_msgs/Marker
      /display_planned_path            moveit_msgs/DisplayTrajectory

    It also subscribes to /display_planned_path so that trajectories planned
    from RViz are also republished to /puma/final_joint_target.
    """

    def __init__(self) -> None:
        super().__init__("puma_pose_to_joint_bridge")

        self.declare_parameter("group_name", "puma_arm")
        self.declare_parameter("end_effector_link", "tool0")
        self.declare_parameter("base_frame", "world")

        self.declare_parameter("input_pose_topic", "/puma/target_pose")
        self.declare_parameter("final_joint_topic", "/puma/final_joint_target")
        self.declare_parameter("status_topic", "/puma/moveit_status")
        self.declare_parameter("marker_topic", "/visualization_marker")
        self.declare_parameter("display_trajectory_topic", "/display_planned_path")
        self.declare_parameter("move_action_name", "/move_action")

        self.declare_parameter("pipeline_id", "ompl")
        self.declare_parameter("planner_id", "")
        self.declare_parameter("allowed_planning_time", 5.0)
        self.declare_parameter("num_planning_attempts", 3)
        self.declare_parameter("max_velocity_scaling_factor", 0.5)
        self.declare_parameter("max_acceleration_scaling_factor", 0.5)
        self.declare_parameter("position_tolerance", 0.01)
        self.declare_parameter("orientation_tolerance", 0.05)

        # False => only plan and publish the result
        # True  => plan and execute
        self.declare_parameter("execute", False)

        self.group_name = self.get_parameter("group_name").value
        self.end_effector_link = self.get_parameter("end_effector_link").value
        self.base_frame = self.get_parameter("base_frame").value
        self.input_pose_topic = self.get_parameter("input_pose_topic").value
        self.final_joint_topic = self.get_parameter("final_joint_topic").value
        self.status_topic = self.get_parameter("status_topic").value
        self.marker_topic = self.get_parameter("marker_topic").value
        self.display_trajectory_topic = self.get_parameter("display_trajectory_topic").value
        self.move_action_name = self.get_parameter("move_action_name").value

        self.pipeline_id = self.get_parameter("pipeline_id").value
        self.planner_id = self.get_parameter("planner_id").value
        self.allowed_planning_time = float(self.get_parameter("allowed_planning_time").value)
        self.num_planning_attempts = int(self.get_parameter("num_planning_attempts").value)
        self.max_velocity_scaling_factor = float(self.get_parameter("max_velocity_scaling_factor").value)
        self.max_acceleration_scaling_factor = float(self.get_parameter("max_acceleration_scaling_factor").value)
        self.position_tolerance = float(self.get_parameter("position_tolerance").value)
        self.orientation_tolerance = float(self.get_parameter("orientation_tolerance").value)
        self.execute = bool(self.get_parameter("execute").value)

        self.final_joint_pub = self.create_publisher(JointState, self.final_joint_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, 10)
        self.display_pub = self.create_publisher(DisplayTrajectory, self.display_trajectory_topic, 10)

        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.input_pose_topic,
            self.on_target_pose,
            10,
        )

        self.display_sub = self.create_subscription(
            DisplayTrajectory,
            self.display_trajectory_topic,
            self.on_display_trajectory,
            10,
        )

        self.move_action_client = ActionClient(self, MoveGroup, self.move_action_name)

        self.publish_status(
            f"Bridge is ready. Listening on {self.input_pose_topic}, "
            f"publishing final joints to {self.final_joint_topic}"
        )

    def publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def publish_target_marker(self, pose: PoseStamped) -> None:
        marker = Marker()
        marker.header = pose.header
        marker.ns = "puma_target"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.scale.z = 0.04
        marker.color.a = 1.0
        marker.color.r = 0.2
        marker.color.g = 0.9
        marker.color.b = 0.2
        self.marker_pub.publish(marker)

    def publish_final_joint_state(self, traj: JointTrajectory, source: str) -> None:
        if not traj.points:
            return

        point = traj.points[-1]
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(traj.joint_names)
        msg.position = list(point.positions)
        msg.velocity = list(point.velocities) if point.velocities else []
        msg.effort = list(point.effort) if point.effort else []
        self.final_joint_pub.publish(msg)

        formatted = ", ".join(
            f"{name}={value:.4f}" for name, value in zip(msg.name, msg.position)
        )
        self.publish_status(f"[{source}] Final joint target: {formatted}")

    def build_goal_constraints(self, pose: PoseStamped) -> Constraints:
        constraints = Constraints()
        constraints.name = "target_pose"

        position_constraint = PositionConstraint()
        position_constraint.header = pose.header
        position_constraint.link_name = self.end_effector_link
        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [
            max(self.position_tolerance * 2.0, 1e-4),
            max(self.position_tolerance * 2.0, 1e-4),
            max(self.position_tolerance * 2.0, 1e-4),
        ]
        position_constraint.constraint_region.primitives.append(primitive)
        position_constraint.constraint_region.primitive_poses.append(pose.pose)
        position_constraint.weight = 1.0

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = pose.header
        orientation_constraint.link_name = self.end_effector_link
        orientation_constraint.orientation = pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = self.orientation_tolerance
        orientation_constraint.absolute_y_axis_tolerance = self.orientation_tolerance
        orientation_constraint.absolute_z_axis_tolerance = self.orientation_tolerance
        orientation_constraint.weight = 1.0

        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)
        return constraints

    def on_target_pose(self, msg: PoseStamped) -> None:
        if not msg.header.frame_id:
            msg.header.frame_id = self.base_frame

        self.publish_target_marker(msg)
        self.publish_status(
            f"[input] Received target pose in frame '{msg.header.frame_id}' "
            f"for link '{self.end_effector_link}'"
        )

        if not self.move_action_client.wait_for_server(timeout_sec=2.0):
            self.publish_status(f"[error] MoveGroup action server '{self.move_action_name}' is not available")
            return

        motion_request = MotionPlanRequest()
        motion_request.group_name = self.group_name
        motion_request.pipeline_id = self.pipeline_id
        motion_request.planner_id = self.planner_id
        motion_request.num_planning_attempts = self.num_planning_attempts
        motion_request.allowed_planning_time = self.allowed_planning_time
        motion_request.max_velocity_scaling_factor = self.max_velocity_scaling_factor
        motion_request.max_acceleration_scaling_factor = self.max_acceleration_scaling_factor
        motion_request.goal_constraints.append(self.build_goal_constraints(msg))

        planning_options = PlanningOptions()
        planning_options.plan_only = not self.execute
        planning_options.look_around = False
        planning_options.replan = False

        goal = MoveGroup.Goal()
        goal.request = motion_request
        goal.planning_options = planning_options

        self.publish_status(
            f"[input] Sending pose to MoveIt (group={self.group_name}, execute={self.execute})"
        )

        send_goal_future = self.move_action_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.publish_status("[error] MoveIt goal was rejected")
            return

        self.publish_status("[input] MoveIt goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_move_group_result)

    def on_move_group_result(self, future) -> None:
        action_result = future.result()
        if action_result is None:
            self.publish_status("[error] MoveIt returned no result")
            return

        result = action_result.result
        error_val = getattr(result.error_code, "val", None)
        if error_val != MoveItErrorCodes.SUCCESS:
            self.publish_status(f"[error] MoveIt failed with code {error_val}")
            return

        traj = result.planned_trajectory.joint_trajectory
        if not traj.points:
            self.publish_status("[error] Empty planned trajectory received from MoveIt")
            return

        self.publish_final_joint_state(traj, source="input_pose")

        display_msg = DisplayTrajectory()
        display_msg.trajectory_start = result.trajectory_start
        display_msg.trajectory.append(result.planned_trajectory)
        self.display_pub.publish(display_msg)

        self.publish_status(
            f"[input_pose] Planned trajectory published to {self.display_trajectory_topic}"
        )

    def on_display_trajectory(self, msg: DisplayTrajectory) -> None:
        if not msg.trajectory:
            return

        joint_traj = msg.trajectory[-1].joint_trajectory
        if not joint_traj.points:
            return

        self.publish_final_joint_state(joint_traj, source="rviz_or_movegroup_display")


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = PumaPoseToJointBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

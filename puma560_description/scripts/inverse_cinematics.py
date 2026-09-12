#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# tf2
from tf2_ros import Buffer, TransformListener
try:
    from tf2_geometry_msgs import do_transform_point  # ROS2 tf2_geometry_msgs
except Exception:
    do_transform_point = None


def rpy_to_R(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    # R = Rz(yaw) * Ry(pitch) * Rx(roll)
    Rz = np.array([[cy, -sy, 0],
                   [sy,  cy, 0],
                   [0,    0, 1]], dtype=float)
    Ry = np.array([[ cp, 0, sp],
                   [  0, 1,  0],
                   [-sp, 0, cp]], dtype=float)
    Rx = np.array([[1,  0,   0],
                   [0, cr, -sr],
                   [0, sr,  cr]], dtype=float)
    return Rz @ Ry @ Rx


def axis_angle_R(axis, theta):
    axis = np.asarray(axis, dtype=float)
    n = np.linalg.norm(axis)
    if n < 1e-12:
        return np.eye(3)
    a = axis / n
    ax, ay, az = a
    ct, st = math.cos(theta), math.sin(theta)
    vt = 1.0 - ct

    return np.array([
        [ct + ax*ax*vt,      ax*ay*vt - az*st, ax*az*vt + ay*st],
        [ay*ax*vt + az*st,   ct + ay*ay*vt,    ay*az*vt - ax*st],
        [az*ax*vt - ay*st,   az*ay*vt + ax*st, ct + az*az*vt],
    ], dtype=float)


def make_T(R, p):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


class Puma560IKTrajectory(Node):
    """
    Subscribes:
      - target_point (geometry_msgs/PointStamped)
      - period_sec   (std_msgs/Float64)
    Publishes:
      - joint_trajectory (trajectory_msgs/JointTrajectory)
    """

    def __init__(self):
        super().__init__("puma560_ik_trajectory")

        # ---- Params ----
        self.declare_parameter("base_frame", "link1")
        self.declare_parameter("target_topic", "/puma560/ik_target")
        self.declare_parameter("period_topic", "/puma560/ik_period_sec")
        self.declare_parameter("traj_topic", "/joint_trajectory_controller/joint_trajectory")

        self.declare_parameter("default_period_sec", 2.0)

        # IK params
        self.declare_parameter("ik_max_iters", 200)
        self.declare_parameter("ik_tol_m", 1e-4)
        self.declare_parameter("ik_damping", 1e-2)
        self.declare_parameter("ik_step", 1.0)

        self.base_frame = self.get_parameter("base_frame").value
        self.target_topic = self.get_parameter("target_topic").value
        self.period_topic = self.get_parameter("period_topic").value
        self.traj_topic = self.get_parameter("traj_topic").value
        self.period_sec = float(self.get_parameter("default_period_sec").value)

        self.ik_max_iters = int(self.get_parameter("ik_max_iters").value)
        self.ik_tol_m = float(self.get_parameter("ik_tol_m").value)
        self.ik_damping = float(self.get_parameter("ik_damping").value)
        self.ik_step = float(self.get_parameter("ik_step").value)

        # ---- TF ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- Kinematic model from YOUR URDF (extracted earlier) ----
        # Each joint: fixed origin xyz/rpy (from parent link to joint frame), and rotation axis (in joint frame)
        # Units: meters, radians.
        self.joint_names = ["j1", "j2", "j3", "j4", "j5", "j6"]

        self.origins_xyz = [
            np.array([0.0, 0.0, 0.6718], dtype=float),            # j1
            np.array([0.0, 0.0, 0.0], dtype=float),               # j2
            np.array([0.4318, -0.0203, 0.1501], dtype=float),     # j3
            np.array([0.0, 0.0, 0.0], dtype=float),               # j4
            np.array([0.0, 0.0, 0.4331], dtype=float),            # j5
            np.array([0.0, 0.0558, 0.0], dtype=float),            # j6
        ]
        self.origins_rpy = [
            np.array([math.pi/2, 0.0, 0.0], dtype=float),         # j1
            np.array([0.0, 0.0, 0.0], dtype=float),               # j2
            np.array([0.0, 0.0, -math.pi/2], dtype=float),        # j3
            np.array([math.pi/2, 0.0, math.pi/2], dtype=float),   # j4
            np.array([math.pi/2, 0.0, 0.0], dtype=float),         # j5
            np.array([-math.pi/2, 0.0, 0.0], dtype=float),        # j6
        ]
        self.axes = [
            np.array([0.0, 1.0, 0.0], dtype=float),               # j1
            np.array([0.0, 0.0, 1.0], dtype=float),               # j2
            np.array([0.0, 0.0, 1.0], dtype=float),               # j3
            np.array([0.0, 0.0, 1.0], dtype=float),               # j4
            np.array([0.0, 0.0, 1.0], dtype=float),               # j5
            np.array([0.0, 0.0, 1.0], dtype=float),               # j6
        ]

        # Joint limits from URDF:
        self.limits = [
            (-math.pi, math.pi),                 # j1
            (-math.pi/2, math.pi/2),             # j2
            (-math.pi/2, math.pi/2),             # j3
            (-math.pi/2, math.pi/2),             # j4
            (-math.pi/2, math.pi/2),             # j5
            (-math.pi/2, math.pi/2),             # j6
        ]

        # seed
        self.q_seed = np.zeros(6, dtype=float)

        # ---- ROS I/O ----
        self.sub_period = self.create_subscription(Float64, self.period_topic, self.on_period, 10)
        self.sub_target = self.create_subscription(PointStamped, self.target_topic, self.on_target, 10)
        self.pub_traj = self.create_publisher(JointTrajectory, self.traj_topic, 10)

        self.get_logger().info(
            f"Ready. target_topic={self.target_topic}, period_topic={self.period_topic}, traj_topic={self.traj_topic}, base_frame={self.base_frame}"
        )

    def on_period(self, msg: Float64):
        if msg.data <= 0.0:
            self.get_logger().warn("Period must be > 0. Ignoring.")
            return
        self.period_sec = float(msg.data)

    def on_target(self, msg: PointStamped):
        # 1) transform to base_frame if possible
        p_base = self.transform_point_to_base(msg)

        if p_base is None:
            self.get_logger().warn("Failed to transform point; ignoring target.")
            return

        p_des = np.array([p_base[0], p_base[1], p_base[2]], dtype=float)

        # 2) IK (position only)
        q_sol, ok, info = self.ik_position_only(p_des, self.q_seed)

        if not ok:
            self.get_logger().warn(f"IK did not converge. pos_err={info['pos_err']:.6f} iters={info['iters']}")
            # Still can publish best-effort if you want; I’ll publish only if converged:
            return

        self.q_seed = q_sol.copy()

        # 3) Publish JointTrajectory
        self.publish_trajectory(q_sol, self.period_sec)
        self.get_logger().info(
            f"Published trajectory. target=({p_des[0]:.3f},{p_des[1]:.3f},{p_des[2]:.3f}) "
            f"period={self.period_sec:.3f}s err={info['pos_err']:.6f} iters={info['iters']}"
        )

    def transform_point_to_base(self, pt_msg: PointStamped):
        # If already in base frame
        if pt_msg.header.frame_id == "" or pt_msg.header.frame_id == self.base_frame:
            return (pt_msg.point.x, pt_msg.point.y, pt_msg.point.z)

        if do_transform_point is None:
            self.get_logger().warn("tf2_geometry_msgs not available; assuming point is already in base_frame.")
            return (pt_msg.point.x, pt_msg.point.y, pt_msg.point.z)

        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                pt_msg.header.frame_id,
                rclpy.time.Time()
            )
            pt_base = do_transform_point(pt_msg, tf)
            return (pt_base.point.x, pt_base.point.y, pt_base.point.z)
        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return None

    # --------- Kinematics ---------

    def fk_and_frames(self, q):
        """
        Returns:
          T_ee (4x4)
          frames: list of dict for each joint i:
            - T_pre: transform up to joint frame BEFORE joint rotation (base->joint)
            - axis_base: joint axis expressed in base coords at that configuration
            - p_joint: joint origin in base coords
        """
        T = np.eye(4)
        frames = []

        for i in range(6):
            xyz = self.origins_xyz[i]
            rpy = self.origins_rpy[i]
            axis_local = self.axes[i]

            R0 = rpy_to_R(rpy[0], rpy[1], rpy[2])
            T_origin = make_T(R0, xyz)

            T_pre = T @ T_origin  # base->joint frame before rotation

            axis_base = T_pre[:3, :3] @ axis_local
            p_joint = T_pre[:3, 3].copy()

            frames.append({"T_pre": T_pre, "axis_base": axis_base, "p_joint": p_joint})

            Rq = axis_angle_R(axis_local, q[i])
            T_rot = make_T(Rq, np.zeros(3))

            T = T_pre @ T_rot  # propagate

        T_ee = T
        return T_ee, frames

    def jacobian_position(self, q):
        T_ee, frames = self.fk_and_frames(q)
        p_ee = T_ee[:3, 3]

        J = np.zeros((3, 6))
        for i in range(6):
            a = frames[i]["axis_base"]
            p = frames[i]["p_joint"]
            J[:, i] = np.cross(a, (p_ee - p))
        return J, p_ee

    def ik_position_only(self, p_des, q0):
        q = np.array(q0, dtype=float).copy()
        lam = float(self.ik_damping)
        step = float(self.ik_step)

        for it in range(self.ik_max_iters):
            J, p_cur = self.jacobian_position(q)
            e = (p_des - p_cur)  # 3

            err = float(np.linalg.norm(e))
            if err < self.ik_tol_m:
                return q, True, {"iters": it, "pos_err": err}

            # DLS: dq = J^T (J J^T + λ^2 I)^-1 e
            JJt = J @ J.T
            A = JJt + (lam * lam) * np.eye(3)
            dq = J.T @ np.linalg.solve(A, e)

            q = q + step * dq

            # clamp limits
            for i in range(6):
                lo, hi = self.limits[i]
                q[i] = float(np.clip(q[i], lo, hi))

        # not converged
        J, p_cur = self.jacobian_position(q)
        err = float(np.linalg.norm(p_des - p_cur))
        return q, False, {"iters": self.ik_max_iters, "pos_err": err}

    def publish_trajectory(self, q, period_sec):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in q]

        sec = int(period_sec)
        nsec = int((period_sec - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=nsec)

        traj.points = [pt]
        self.pub_traj.publish(traj)


def main():
    rclpy.init()
    node = Puma560IKTrajectory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

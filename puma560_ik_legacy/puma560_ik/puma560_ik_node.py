#!/usr/bin/env python3
import math
import numpy as np
import rclpy
import  time
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# ---------- math helpers ----------

def rpy_to_R(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr]
    ], dtype=float)


def skew(w):
    wx, wy, wz = w
    return np.array([
        [0.0, -wz,  wy],
        [wz,   0.0, -wx],
        [-wy,  wx,  0.0]
    ], dtype=float)


def rot_log(R):
    tr = float(np.trace(R))
    cos_theta = (tr - 1.0) / 2.0
    cos_theta = max(-1.0, min(1.0, cos_theta))
    theta = math.acos(cos_theta)
    if theta < 1e-9:
        return np.zeros(3, dtype=float)

    s = math.sin(theta)
    if abs(s) < 1e-9:
        return np.zeros(3, dtype=float)

    W = (R - R.T) / (2.0 * s)
    return theta * np.array([W[2, 1], W[0, 2], W[1, 0]], dtype=float)


def T_from_Rp(R, p):
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def T_from_xyz_rpy(xyz, rpy):
    R = rpy_to_R(rpy[0], rpy[1], rpy[2])
    return T_from_Rp(R, np.array(xyz, dtype=float))


def axis_angle_to_R(axis, angle):
    axis = np.array(axis, dtype=float)
    n = float(np.linalg.norm(axis))
    if n < 1e-12:
        return np.eye(3, dtype=float)
    a = axis / n
    K = skew(a)
    return np.eye(3, dtype=float) + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)

def wrap_to_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi
# ---------- chain model from parameters ----------

class Chain6R:
    def __init__(self, origin_xyz, origin_rpy, axes, lower, upper):
        self.n = 6
        self.origin_xyz = [np.array(v, dtype=float) for v in origin_xyz]
        self.origin_rpy = [np.array(v, dtype=float) for v in origin_rpy]
        self.axes = [np.array(v, dtype=float) for v in axes]
        self.lower = np.array(lower, dtype=float)
        self.upper = np.array(upper, dtype=float)

    def fk_and_jacobian(self, q):
        T = np.eye(4, dtype=float)

        z_list = []
        p_list = []

        for i in range(self.n):
            T = T @ T_from_xyz_rpy(self.origin_xyz[i], self.origin_rpy[i])

            R_base_joint = T[:3, :3]
            axis_base = R_base_joint @ self.axes[i]
            z_list.append(axis_base)
            p_list.append(T[:3, 3].copy())

            Rj = axis_angle_to_R(self.axes[i], float(q[i]))
            T = T @ T_from_Rp(Rj, np.zeros(3, dtype=float))

        p_tip = T[:3, 3]
        Jv = np.zeros((3, self.n), dtype=float)
        Jw = np.zeros((3, self.n), dtype=float)

        for i in range(self.n):
            z = z_list[i]
            p_i = p_list[i]
            Jw[:, i] = z
            Jv[:, i] = np.cross(z, (p_tip - p_i))

        J = np.vstack([Jv, Jw])
        return T, J


class Puma560IKNode(Node):
    def __init__(self):
        super().__init__("puma560_ik_node")

        self.declare_parameter("input_topic", "/puma560/target_xyz_period")
        self.declare_parameter("output_topic", "/joint_trajectory_controller/joint_trajectory")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("joint_names", ["j1", "j2", "j3", "j4", "j5", "j6"])

        self.declare_parameter("max_iters", 400)
        self.declare_parameter("damping", 5e-2)
        self.declare_parameter("step", 0.5)
        self.declare_parameter("tol_pos", 1e-4)
        self.declare_parameter("tol_rot", 0.15)
        self.declare_parameter("use_orientation", False)
        self.declare_parameter("desired_rpy", [0.0, 0.0, 0.0])
        self.declare_parameter("w_rot", 0.05)

        # hand-aligned reference offset
        self.declare_parameter("hand_frame_roll_deg", 0.0)
        self.declare_parameter("hand_frame_pitch_deg", 0.0)
        self.declare_parameter("hand_frame_yaw_deg", 0.0)

        # Tool / wrist-center parameters
        self.declare_parameter("tool_offset_local", [0.0, 0.0, 0.0])

        # DH-like geometric approximations for analytical position-stage seeds
        self.declare_parameter("dh_h", 0.6718)
        self.declare_parameter("dh_l2", 0.4318)
        self.declare_parameter("dh_b3", 0.0203)
        self.declare_parameter("dh_l3", 0.1501)
        self.declare_parameter("dh_l4", 0.4331)
        self.declare_parameter("dh_dwt", 0.0558)

        self.declare_parameter("joint_limit_tol", 1e-6)
        self.declare_parameter("workspace_min_z", 0.05)
        self.declare_parameter("workspace_max_radius", 1.50)
        self.declare_parameter("workspace_min_radius", 0.10)
        self.declare_parameter("workspace_fk_tol", 5e-3)

        self.declare_parameter("origin_xyz_flat", [
            0.0, 0.0, 0.6718,
            0.0, 0.0, 0.0,
            0.4318, -0.0203, 0.1501,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.4331,
            0.0, 0.0558, 0.0,
        ])
        self.declare_parameter("origin_rpy_flat", [
            math.pi / 2, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, -math.pi / 2,
            math.pi / 2, 0.0, math.pi / 2,
            math.pi / 2, 0.0, 0.0,
            -math.pi / 2, 0.0, 0.0,
        ])
        self.declare_parameter("axis_flat", [
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
            0.0, 0.0, 1.0,
            0.0, 0.0, 1.0,
            0.0, 0.0, 1.0,
            0.0, 0.0, 1.0,
        ])
        self.declare_parameter(
            "lower",
            [-math.pi, -math.pi / 2, -math.pi / 2, -math.pi / 2, -math.pi / 2, -math.pi / 2]
        )
        self.declare_parameter(
            "upper",
            [math.pi, math.pi / 2, math.pi / 2, math.pi / 2, math.pi / 2, math.pi / 2]
        )

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.joint_state_topic = self.get_parameter("joint_state_topic").value
        self.joint_names = list(self.get_parameter("joint_names").value)

        self.joint_limit_tol = float(self.get_parameter("joint_limit_tol").value)
        self.workspace_min_z = float(self.get_parameter("workspace_min_z").value)
        self.workspace_max_radius = float(self.get_parameter("workspace_max_radius").value)
        self.workspace_min_radius = float(self.get_parameter("workspace_min_radius").value)
        self.workspace_fk_tol = float(self.get_parameter("workspace_fk_tol").value)

        self.hand_frame_roll_deg = float(self.get_parameter("hand_frame_roll_deg").value)
        self.hand_frame_pitch_deg = float(self.get_parameter("hand_frame_pitch_deg").value)
        self.hand_frame_yaw_deg = float(self.get_parameter("hand_frame_yaw_deg").value)

        self.tool_offset_local = np.array(
            self.get_parameter("tool_offset_local").value,
            dtype=float
        )
        if self.tool_offset_local.shape != (3,):
            raise RuntimeError("tool_offset_local must be length 3")

        self.dh_h = float(self.get_parameter("dh_h").value)
        self.dh_l2 = float(self.get_parameter("dh_l2").value)
        self.dh_b3 = float(self.get_parameter("dh_b3").value)
        self.dh_l3 = float(self.get_parameter("dh_l3").value)
        self.dh_l4 = float(self.get_parameter("dh_l4").value)
        self.dh_dwt = float(self.get_parameter("dh_dwt").value)

        if len(self.joint_names) != 6:
            raise RuntimeError("joint_names must be length 6")

        origin_xyz = self._unflatten_6x3(self.get_parameter("origin_xyz_flat").value, "origin_xyz_flat")
        origin_rpy = self._unflatten_6x3(self.get_parameter("origin_rpy_flat").value, "origin_rpy_flat")
        axes = self._unflatten_6x3(self.get_parameter("axis_flat").value, "axis_flat")

        lower = self.get_parameter("lower").value
        upper = self.get_parameter("upper").value
        if len(lower) != 6 or len(upper) != 6:
            raise RuntimeError("lower/upper must be length 6")

        self.chain = Chain6R(origin_xyz, origin_rpy, axes, lower, upper)

        self.q_current = np.zeros(6, dtype=float)
        self.have_joint_state = False

        self.sub_js = self.create_subscription(JointState, self.joint_state_topic, self.on_joint_state, 10)
        self.sub_in = self.create_subscription(Float64MultiArray, self.input_topic, self.on_target, 10)
        self.pub_traj = self.create_publisher(JointTrajectory, self.output_topic, 10)
        self.status_pub = self.create_publisher(String, "/puma560/status", 10)

        self.get_logger().info(f"IK node ready. input={self.input_topic} output={self.output_topic}")

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _unflatten_6x3(self, arr, name):
        if len(arr) != 18:
            raise RuntimeError(f"{name} must have length 18 (6x3)")
        out = []
        for i in range(6):
            out.append([
                float(arr[3 * i + 0]),
                float(arr[3 * i + 1]),
                float(arr[3 * i + 2]),
            ])
        return out

    def is_within_joint_limits(self, q):
        q = np.array(q, dtype=float)
        lower_ok = q >= (self.chain.lower - self.joint_limit_tol)
        upper_ok = q <= (self.chain.upper + self.joint_limit_tol)
        return bool(np.all(lower_ok & upper_ok))

    def joint_limit_violations(self, q):
        q = np.array(q, dtype=float)
        violations = []
        for i in range(6):
            lo = self.chain.lower[i] - self.joint_limit_tol
            hi = self.chain.upper[i] + self.joint_limit_tol
            if q[i] < lo or q[i] > hi:
                violations.append(
                    f"{self.joint_names[i]}={q[i]:.6f} rad not in "
                    f"[{self.chain.lower[i]:.6f}, {self.chain.upper[i]:.6f}]"
                )
        return violations

    def is_point_in_coarse_workspace(self, p_point):
        """
        Coarse workspace check.
        3D point vector, intended to be the WCP.
        """
        x, y, z = map(float, p_point)

        if not all(math.isfinite(v) for v in [x, y, z]):
            return False, "Non-finite coordinates."

        if z < self.workspace_min_z:
            return False, f"z={z:.3f} is below workspace_min_z={self.workspace_min_z:.3f}"

        r = math.sqrt(x * x + y * y + z * z)

        if r > self.workspace_max_radius:
            return False, f"Target radius {r:.3f} exceeds workspace_max_radius={self.workspace_max_radius:.3f}"

        if r < self.workspace_min_radius:
            return False, f"Target radius {r:.3f} is below workspace_min_radius={self.workspace_min_radius:.3f}"

        return True, ""

    def is_solution_valid(self, q_sol, p_target):
        if not self.is_within_joint_limits(q_sol):
            return False, "IK solution violates joint limits."

        T_chk, _ = self.chain.fk_and_jacobian(q_sol)
        p_chk = T_chk[:3, 3]
        err = float(np.linalg.norm(np.array(p_target, dtype=float) - p_chk))

        if err > self.workspace_fk_tol:
            return False, f"FK verification error too large: {err:.6f} m"

        return True, ""

    def is_target_reachable(self, x, y, z, q_seed, target_orientation=None):
        """
        Reachability check uses:
        1) desired TCP pose
        2) computed WCP
        3) coarse workspace check on WCP
        4) IK solve
        5) FK endpoint validation on TCP
        """

        p_tcp = np.array([x, y, z], dtype=float)

        if target_orientation is None:
            R_tcp = self.get_default_tcp_rotation()
        else:
            R_tcp = target_orientation

        p_wcp = self.compute_wcp(p_tcp, R_tcp)
        self.get_logger().info(
            f"Target TCP=({p_tcp[0]:.4f}, {p_tcp[1]:.4f}, {p_tcp[2]:.4f}) "
            f"WCP=({p_wcp[0]:.4f}, {p_wcp[1]:.4f}, {p_wcp[2]:.4f})"
        )
        ok_ws, reason_ws = self.is_point_in_coarse_workspace(p_wcp)
        if not ok_ws:
            return False, None, f"WCP workspace check failed: {reason_ws}"

        # q_sol, ok_ik = self.solve_ik(
        #     p_tcp,
        #     q_seed,
        #     target_orientation=target_orientation
        # )
        # if not ok_ik:
        #     return False, None, "IK did not converge."

        # First solve the position stage through WCP
        q_pos, ok_pos = self.solve_position_stage(p_wcp, q_seed)
        self.get_logger().info(
            "Position-stage q = " +
            ", ".join(f"{name}={val:.4f}" for name, val in zip(self.joint_names, q_pos))
        )
        if not ok_pos:
            # dt = time.perf_counter() - t0
            # self.get_logger().info(f"Position-stage failed after {dt:.3f} s")
            return False, None, "Position-stage IK did not converge."

        # If no orientation is requested, position-stage result is enough for now
        if target_orientation is None:
            q_sol = q_pos
        else:
            q_sol, ok_pose = self.solve_pose_stage(
                p_tcp,
                q_pos,
                target_orientation
            )
            if not ok_pose:
                return False, None, "Full pose IK did not converge."

        ok_sol, reason_sol = self.is_solution_valid(q_sol, p_tcp)
        if not ok_sol:
            return False, None, reason_sol

        return True, q_sol, ""

    def on_joint_state(self, msg: JointState):
        idx = {n: i for i, n in enumerate(msg.name)}
        q = np.zeros(6, dtype=float)
        for i, jn in enumerate(self.joint_names):
            if jn not in idx:
                return
            q[i] = float(msg.position[idx[jn]])
        self.q_current = q
        self.have_joint_state = True

    def build_target_rotation(self, roll_rad, pitch_rad, yaw_rad, orientation_mode):
        R_gui = rpy_to_R(roll_rad, pitch_rad, yaw_rad)

        if orientation_mode == "hand":
            R_offset = rpy_to_R(
                math.radians(self.hand_frame_roll_deg),
                math.radians(self.hand_frame_pitch_deg),
                math.radians(self.hand_frame_yaw_deg)
            )
            return R_offset @ R_gui

        return R_gui

    def compute_wcp(self, p_tcp, R_tcp):
        """
        Convert desired TCP position/orientation to wrist center point (WCP).

        p_tcp: np.array shape (3,)
        R_tcp: np.array shape (3,3)

        Returns:
            p_wcp: np.array shape (3,)
        """
        return np.array(p_tcp, dtype=float) - R_tcp @ self.tool_offset_local

    def get_default_tcp_rotation(self):
        """
        Default TCP rotation used when orientation is not explicitly requested.
        For now, use identity / base-aligned frame.
        Later this can be replaced with hand-aligned default if needed.
        """
        return np.eye(3, dtype=float)

    def get_tcp_pose_from_q(self, q):
        """
        Returns current TCP pose from FK.
        For now FK tip is treated as TCP.
        Later this function can be extended if TCP frame differs from chain tip.
        """
        T, _ = self.chain.fk_and_jacobian(q)
        p_tcp = T[:3, 3].copy()
        R_tcp = T[:3, :3].copy()
        return p_tcp, R_tcp

    def get_R03_from_q123(self, q):
        """
        Approximate arm rotation up to joint 3 by setting wrist joints to zero.
        Returns rotation matrix R03.
        """
        q_arm = np.array(q, dtype=float).copy()
        q_arm[3:] = 0.0
        T03, _ = self.chain.fk_and_jacobian(q_arm)
        return T03[:3, :3].copy()

    def solve_position_to_wcp_analytic_seeds(self, p_wcp, q_seed):
        """
        Improved analytical seed generator for q1,q2,q3.

        Uses a planar arm approximation in the rho-z plane:
          q1 from atan2(y, x)
          q2, q3 from 2-link geometry

        This is still not the final exact PUMA closed-form IK, but it is more stable
        than the previous version and produces fewer bad branches.
        """
        p_wcp = np.array(p_wcp, dtype=float)
        q_seed = np.array(q_seed, dtype=float).copy()

        # Shoulder/base reference
        shoulder = np.array([0.0, 0.0, self.dh_h], dtype=float)
        rel = p_wcp - shoulder
        px, py, pz = rel

        rho = math.hypot(px, py)
        z = pz

        # Main shoulder yaw candidate only
        q1 = wrap_to_pi(math.atan2(py, px))

        # Effective planar lengths
        # Use the same approximations, but in a cleaner way
        L1 = float(np.linalg.norm(np.array([self.dh_l2, self.dh_b3], dtype=float)))
        L2 = float(np.linalg.norm(np.array([self.dh_l3, self.dh_l4], dtype=float)))

        r2 = rho * rho + z * z
        cos_q3 = (r2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)

        if cos_q3 < -1.0 or cos_q3 > 1.0:
            return []

        cos_q3 = max(-1.0, min(1.0, cos_q3))
        sin_q3_abs = math.sqrt(max(0.0, 1.0 - cos_q3 * cos_q3))

        base_candidates = []

        # Only elbow-up / elbow-down branches
        for sin_q3 in (+sin_q3_abs, -sin_q3_abs):
            q3 = wrap_to_pi(math.atan2(sin_q3, cos_q3))

            k1 = L1 + L2 * cos_q3
            k2 = L2 * sin_q3

            q2 = wrap_to_pi(math.atan2(z, rho) - math.atan2(k2, k1))

            q_candidate = q_seed.copy()
            q_candidate[0] = q1
            q_candidate[1] = q2
            q_candidate[2] = q3
            q_candidate = np.clip(q_candidate, self.chain.lower, self.chain.upper)

            base_candidates.append(q_candidate)

        # Small local perturbations only
        delta_q2_list = [0.0, math.radians(5.0), math.radians(-5.0)]
        delta_q3_list = [0.0, math.radians(5.0), math.radians(-5.0)]

        expanded_candidates = []
        for qc in base_candidates:
            for dq2 in delta_q2_list:
                for dq3 in delta_q3_list:
                    q_var = qc.copy()
                    q_var[1] = wrap_to_pi(q_var[1] + dq2)
                    q_var[2] = wrap_to_pi(q_var[2] + dq3)
                    q_var = np.clip(q_var, self.chain.lower, self.chain.upper)
                    expanded_candidates.append(q_var)

        # Remove near-duplicates
        unique_candidates = []
        for qc in expanded_candidates:
            is_duplicate = False
            for qu in unique_candidates:
                if np.linalg.norm(qc[:3] - qu[:3]) < 1e-4:
                    is_duplicate = True
                    break
            if not is_duplicate:
                unique_candidates.append(qc)

        return unique_candidates

    def solve_position_to_wcp_numeric(self, p_wcp, q0):
        """
        Temporary numeric solver for the position stage.
        Works only with position error and is meant as a clean placeholder
        before replacing it with analytical IK for q1,q2,q3.
        """
        max_iters = int(self.get_parameter("max_iters").value)
        damping = float(self.get_parameter("damping").value)
        step = float(self.get_parameter("step").value)
        tol_pos = float(self.get_parameter("tol_pos").value)

        q = np.array(q0, dtype=float).copy()
        q = np.clip(q, self.chain.lower, self.chain.upper)

        I6 = np.eye(6, dtype=float)

        for _ in range(max_iters):
            p_tcp_cur, R_tcp_cur = self.get_tcp_pose_from_q(q)
            p_wcp_cur = self.compute_wcp(p_tcp_cur, R_tcp_cur)

            e_p = np.array(p_wcp, dtype=float) - p_wcp_cur

            if np.linalg.norm(e_p) < tol_pos:
                return q, True

            _, J = self.chain.fk_and_jacobian(q)
            J_pos = J[:3, :]

            A = (J_pos @ J_pos.T) + (damping ** 2) * np.eye(3, dtype=float)
            try:
                dq = J_pos.T @ np.linalg.solve(A, e_p)
            except np.linalg.LinAlgError:
                return q, False

            q = q + step * dq
            q = np.clip(q, self.chain.lower, self.chain.upper)

        return q, False

    def score_joint_seed(self, q_candidate, q_reference):
        """
        Lower score = better seed.
        We prefer seeds close to the current configuration.
        """
        q_candidate = np.array(q_candidate, dtype=float)
        q_reference = np.array(q_reference, dtype=float)
        dq = q_candidate - q_reference
        return float(np.linalg.norm(dq))

    def solve_position_stage(self, p_wcp, q0):
        """
        Position stage:
        1) build analytical q1,q2,q3 seeds
        2) refine each candidate numerically on WCP
        3) if all analytical candidates fail, fallback to pure numeric from q0
        4) choose the best successful solution
        """
        candidates = self.solve_position_to_wcp_analytic_seeds(p_wcp, q0)

        candidates = sorted(
            candidates,
            key=lambda qc: self.score_joint_seed(qc, q0)
        )
        candidates = candidates[:6]

        best_q = None
        best_err = float("inf")

        # First try analytical-seed-based refinement
        for q_seed_candidate in candidates:
            q_refined, ok = self.solve_position_to_wcp_numeric(p_wcp, q_seed_candidate)
            if not ok:
                continue

            p_tcp_chk, R_tcp_chk = self.get_tcp_pose_from_q(q_refined)
            p_wcp_chk = self.compute_wcp(p_tcp_chk, R_tcp_chk)
            err = float(np.linalg.norm(np.array(p_wcp, dtype=float) - p_wcp_chk))

            if err < best_err:
                best_err = err
                best_q = q_refined

        # robust fallback to old pure numeric path from current seed
        q_num, ok_num = self.solve_position_to_wcp_numeric(p_wcp, q0)
        if ok_num:
            p_tcp_chk, R_tcp_chk = self.get_tcp_pose_from_q(q_num)
            p_wcp_chk = self.compute_wcp(p_tcp_chk, R_tcp_chk)
            err = float(np.linalg.norm(np.array(p_wcp, dtype=float) - p_wcp_chk))

            if err < best_err:
                best_err = err
                best_q = q_num

        if best_q is None:
            return np.array(q0, dtype=float).copy(), False

        return best_q, True

    def solve_wrist_analytic_seeds(self, q_pos, target_orientation):
        """
        Analytical wrist seed generator for q4,q5,q6.

        Uses:
          R36 = R03^T * R_des

        and a simple spherical-wrist-style decomposition to generate wrist seeds.
        This is an approximation for the current URDF-like chain, not yet an exact
        closed-form wrist IK for this model.
        """
        q_pos = np.array(q_pos, dtype=float).copy()
        R03 = self.get_R03_from_q123(q_pos)
        R36 = R03.T @ target_orientation

        # Clamp for numerical stability
        c5 = max(-1.0, min(1.0, float(R36[2, 2])))

        candidates = []

        # Two wrist branches
        for q5 in (math.acos(c5), -math.acos(c5)):
            s5 = math.sin(q5)

            if abs(s5) < 1e-6:
                # Wrist singularity-like case:
                # collapse q4/q6 into a simple fallback seed
                q4 = 0.0
                q6 = math.atan2(-R36[0, 1], R36[0, 0])
            else:
                q4 = math.atan2(R36[1, 2] / (-s5), R36[0, 2] / (-s5))
                q6 = math.atan2(R36[2, 1] / (-s5), R36[2, 0] / s5)

            q_seed = q_pos.copy()
            q_seed[3] = wrap_to_pi(q4)
            q_seed[4] = wrap_to_pi(q5)
            q_seed[5] = wrap_to_pi(q6)
            q_seed = np.clip(q_seed, self.chain.lower, self.chain.upper)
            candidates.append(q_seed)

        # Add tiny local perturbations for robustness
        delta_list = [0.0, math.radians(5.0), math.radians(-5.0)]
        expanded = []
        for qc in candidates:
            for dq4 in delta_list:
                for dq5 in [0.0]:
                    for dq6 in delta_list:
                        qv = qc.copy()
                        qv[3] = wrap_to_pi(qv[3] + dq4)
                        qv[4] = wrap_to_pi(qv[4] + dq5)
                        qv[5] = wrap_to_pi(qv[5] + dq6)
                        qv = np.clip(qv, self.chain.lower, self.chain.upper)
                        expanded.append(qv)

        # Remove near-duplicates
        unique_candidates = []
        for qc in expanded:
            is_duplicate = False
            for qu in unique_candidates:
                if np.linalg.norm(qc[3:] - qu[3:]) < 1e-4:
                    is_duplicate = True
                    break
            if not is_duplicate:
                unique_candidates.append(qc)

        return unique_candidates

    def solve_pose_stage(self, p_tcp, q_pos, target_orientation):
        """
        Full pose stage:
        1) build analytical wrist seeds from q_pos
        2) refine each candidate with full numeric pose IK
        3) fallback to old full numeric IK from q_pos
        4) choose the best successful solution
        """
        candidates = self.solve_wrist_analytic_seeds(q_pos, target_orientation)

        candidates = sorted(
            candidates,
            key=lambda qc: self.score_joint_seed(qc, q_pos)
        )
        candidates = candidates[:6]

        best_q = None
        best_score = float("inf")

        self.get_logger().info(f"Full-pose wrist candidates: {len(candidates)}")

        # Try analytic wrist seeds first
        for q_seed_candidate in candidates:
            q_refined, ok = self.solve_ik(
                p_tcp,
                q_seed_candidate,
                target_orientation=target_orientation
            )
            if not ok:
                continue

            p_chk, R_chk = self.get_tcp_pose_from_q(q_refined)
            pos_err = float(np.linalg.norm(np.array(p_tcp, dtype=float) - p_chk))
            rot_err = float(np.linalg.norm(rot_log(target_orientation @ R_chk.T)))

            score = pos_err + 0.1 * rot_err
            if score < best_score:
                best_score = score
                best_q = q_refined

        # Robust fallback: old full numeric path from q_pos
        q_num, ok_num = self.solve_ik(
            p_tcp,
            q_pos,
            target_orientation=target_orientation
        )
        if ok_num:
            p_chk, R_chk = self.get_tcp_pose_from_q(q_num)
            pos_err = float(np.linalg.norm(np.array(p_tcp, dtype=float) - p_chk))
            rot_err = float(np.linalg.norm(rot_log(target_orientation @ R_chk.T)))

            score = pos_err + 0.1 * rot_err
            if score < best_score:
                best_score = score
                best_q = q_num

        if best_q is None:
            return np.array(q_pos, dtype=float).copy(), False

        self.get_logger().info(f"Best full-pose score: {best_score:.6f}")
        return best_q, True

    def solve_ik(self, p_des, q0, target_orientation=None):
        max_iters = int(self.get_parameter("max_iters").value)
        damping = float(self.get_parameter("damping").value)
        step = float(self.get_parameter("step").value)
        tol_pos = float(self.get_parameter("tol_pos").value)
        tol_rot = float(self.get_parameter("tol_rot").value)
        use_orientation_param = bool(self.get_parameter("use_orientation").value)
        w_rot = float(self.get_parameter("w_rot").value)

        use_orientation = (target_orientation is not None) or use_orientation_param

        if target_orientation is not None:
            R_des = target_orientation
        elif use_orientation_param:
            drpy = self.get_parameter("desired_rpy").value
            R_des = rpy_to_R(float(drpy[0]), float(drpy[1]), float(drpy[2]))
        else:
            R_des = None

        q = np.array(q0, dtype=float).copy()
        I6 = np.eye(6, dtype=float)
        q = np.clip(q, self.chain.lower, self.chain.upper)

        for _ in range(max_iters):
            T_cur, J = self.chain.fk_and_jacobian(q)
            p_cur = T_cur[:3, 3]
            e_p = np.array(p_des, dtype=float) - p_cur

            if use_orientation:
                R_cur = T_cur[:3, :3]
                e_w = rot_log(R_des @ R_cur.T)
            else:
                e_w = np.zeros(3, dtype=float)

            # relaxed convergence
            if np.linalg.norm(e_p) < tol_pos:
                if not use_orientation:
                    return q, True
                if np.linalg.norm(e_w) < tol_rot:
                    return q, True

            e = np.hstack([e_p, w_rot * e_w])

            A = (J @ J.T) + (damping ** 2) * I6
            try:
                dq = J.T @ np.linalg.solve(A, e)
            except np.linalg.LinAlgError:
                return q, False

            q = q + step * dq
            q = np.clip(q, self.chain.lower, self.chain.upper)

        return q, False

    def on_target(self, msg: Float64MultiArray):
        try:
            t0 = time.perf_counter()
            n = len(msg.data)

            # 4 values: x y z period
            # 8 values: x y z roll pitch yaw mode_code period
            if n not in (4, 8):
                text = "ERROR: Input must be [x,y,z,period_sec] or [x,y,z,roll_deg,pitch_deg,yaw_deg,mode_code,period_sec]."
                self.get_logger().warn(text)
                self.publish_status(text)
                return

            if not all(math.isfinite(v) for v in msg.data):
                text = "ERROR: Input contains non-finite values."
                self.get_logger().warn(text)
                self.publish_status(text)
                return

            if n == 4:
                x, y, z, period = map(float, msg.data[:4])
                target_orientation = None
            else:
                x, y, z, roll_deg, pitch_deg, yaw_deg, mode_code, period = map(float, msg.data[:8])

                orientation_mode = "world" if int(round(mode_code)) == 0 else "hand"

                target_orientation = self.build_target_rotation(
                    math.radians(roll_deg),
                    math.radians(pitch_deg),
                    math.radians(yaw_deg),
                    orientation_mode
                )

            if period <= 0.0:
                text = "ERROR: period_sec must be > 0."
                self.get_logger().warn(text)
                self.publish_status(text)
                return

            q0 = self.q_current if self.have_joint_state else np.zeros(6, dtype=float)

            reachable, q_sol, reason = self.is_target_reachable(
                x, y, z, q0, target_orientation=target_orientation
            )
            if not reachable:
                text = f"ERROR: {reason}"
                self.get_logger().warn(text)
                self.publish_status(text)
                return

            if not self.is_within_joint_limits(q_sol):
                details = "; ".join(self.joint_limit_violations(q_sol))
                text = f"ERROR: IK solution violates joint limits. Command rejected. {details}"
                self.get_logger().warn(text)
                self.publish_status(text)
                return

            traj = JointTrajectory()
            traj.joint_names = self.joint_names

            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in q_sol]

            secs = int(period)
            nsecs = int(round((period - secs) * 1e9))
            if nsecs >= 1_000_000_000:
                secs += 1
                nsecs -= 1_000_000_000

            pt.time_from_start = Duration(seconds=secs, nanoseconds=nsecs).to_msg()

            traj.points = [pt]
            dt = time.perf_counter() - t0
            self.get_logger().info(
                "Final q = " +
                ", ".join(f"{name}={val:.4f}" for name, val in zip(self.joint_names, q_sol))
            )
            self.get_logger().info(f"Solve time = {dt:.3f} s")
            self.pub_traj.publish(traj)

            if target_orientation is None:
                text = "OK: Trajectory published (position only)."
            else:
                text = "OK: Trajectory published (position + orientation)."

            self.publish_status(text)
            self.get_logger().info(
                text + " " +
                ", ".join(f"{name}={val:.4f}" for name, val in zip(self.joint_names, q_sol))
            )

        except Exception as e:
            text = f"ERROR: Exception in on_target: {e}"
            self.get_logger().error(text)
            self.publish_status(text)


def main():
    rclpy.init()
    node = Puma560IKNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
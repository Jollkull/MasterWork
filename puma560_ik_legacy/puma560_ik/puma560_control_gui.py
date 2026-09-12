#!/usr/bin/env python3
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


PRESETS = {
    "Home":  [0.45,  0.00, 0.55, 3.0],
    "Front": [0.50,  0.00, 0.50, 3.0],
    "Left":  [0.40,  0.10, 0.55, 3.0],
    "Right": [0.40, -0.10, 0.55, 3.0],
    "Upper": [0.35,  0.00, 0.70, 4.0],
    "Wave":  None,
}


class PumaGuiNode(Node):
    def __init__(self):
        super().__init__("puma560_control_gui")

        self.coord_pub = self.create_publisher(
            Float64MultiArray,
            "/puma560/target_xyz_period",
            10
        )

        self.traj_pub = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.on_joint_state,
            10
        )

        self.status_sub = self.create_subscription(
            String,
            "/puma560/status",
            self.on_status,
            10
        )

        self.joint_names = ["j1", "j2", "j3", "j4", "j5", "j6"]
        self.current_positions = None

        self.status_callback = None

        self.wave_active = False
        self.wave_thread = None

    def on_joint_state(self, msg: JointState):
        idx = {name: i for i, name in enumerate(msg.name)}
        try:
            self.current_positions = [float(msg.position[idx[j]]) for j in self.joint_names]
        except KeyError:
            return

    def on_status(self, msg: String):
        if self.status_callback is not None:
            self.status_callback(msg.data)

    def publish_coordinates(
        self,
        x: float,
        y: float,
        z: float,
        roll_deg: float,
        pitch_deg: float,
        yaw_deg: float,
        period: float,
        use_orientation: bool,
        orientation_mode: str
    ):
        self.stop_wave()
        msg = Float64MultiArray()

        if use_orientation:
            mode_code = 0.0 if orientation_mode == "World" else 1.0
            msg.data = [
                float(x), float(y), float(z),
                float(roll_deg), float(pitch_deg), float(yaw_deg),
                float(mode_code),
                float(period)
            ]
            self.get_logger().info(
                f"Published target: x={x:.3f}, y={y:.3f}, z={z:.3f}, "
                f"roll={roll_deg:.1f}, pitch={pitch_deg:.1f}, yaw={yaw_deg:.1f}, "
                f"mode={orientation_mode}, t={period:.3f}"
            )
        else:
            msg.data = [float(x), float(y), float(z), float(period)]
            self.get_logger().info(
                f"Published target: x={x:.3f}, y={y:.3f}, z={z:.3f}, t={period:.3f}"
            )

        self.coord_pub.publish(msg)

    def publish_joint_positions(self, positions, duration_sec):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in positions]

        secs = int(duration_sec)
        nsecs = int(round((duration_sec - secs) * 1e9))
        if nsecs >= 1_000_000_000:
            secs += 1
            nsecs -= 1_000_000_000

        pt.time_from_start = Duration(seconds=secs, nanoseconds=nsecs).to_msg()

        traj.points = [pt]
        self.traj_pub.publish(traj)

    def stop_wave(self):
        self.wave_active = False

    def start_wave(self):
        if self.wave_active:
            return

        self.wave_active = True
        self.wave_thread = threading.Thread(target=self._wave_loop, daemon=True)
        self.wave_thread.start()

    def _wave_loop(self):
        base_pose = [0.0, 0.9, -1.2, 0.0, 0.2, 0.0]
        wave_left =  [0.0, 0.9, -1.2, 0.0, 0.2, -0.6]
        wave_right = [0.0, 0.9, -1.2, 0.0, 0.2,  0.6]

        self.publish_joint_positions(base_pose, 2.0)
        time.sleep(2.2)

        while self.wave_active:
            self.publish_joint_positions(wave_left, 0.8)
            time.sleep(0.9)
            if not self.wave_active:
                break

            self.publish_joint_positions(wave_right, 0.8)
            time.sleep(0.9)

    def abort_motion(self):
        self.stop_wave()

        if self.current_positions is None:
            self.get_logger().warn("Abort requested, but no joint state received yet.")
            return False

        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        pt = JointTrajectoryPoint()
        pt.positions = list(self.current_positions)
        pt.time_from_start = Duration(seconds=0, nanoseconds=200_000_000).to_msg()

        traj.points = [pt]
        self.traj_pub.publish(traj)

        self.get_logger().warn("Abort command sent: holding current joint positions.")
        return True


class PumaControlGUI:
    def __init__(self, node: PumaGuiNode):
        self.node = node
        self.node.status_callback = self.handle_status

        self.root = tk.Tk()
        self.root.title("PUMA560 IK Control")
        self.root.geometry("480x700")
        self.root.resizable(True, True)

        self._build_ui()

    def _build_ui(self):
        main = ttk.Frame(self.root, padding=12)
        main.pack(fill="both", expand=True)

        ttk.Label(
            main,
            text="PUMA560 IK Control Panel",
            font=("Arial", 14, "bold")
        ).pack(pady=(0, 12))

        coord_frame = ttk.LabelFrame(main, text="Manual coordinates + orientation", padding=10)
        coord_frame.pack(fill="x", pady=6)

        ttk.Label(coord_frame, text="X").grid(row=0, column=0, sticky="w", padx=4, pady=4)
        ttk.Label(coord_frame, text="Y").grid(row=1, column=0, sticky="w", padx=4, pady=4)
        ttk.Label(coord_frame, text="Z").grid(row=2, column=0, sticky="w", padx=4, pady=4)
        ttk.Label(coord_frame, text="Roll (deg)").grid(row=3, column=0, sticky="w", padx=4, pady=4)
        ttk.Label(coord_frame, text="Pitch (deg)").grid(row=4, column=0, sticky="w", padx=4, pady=4)
        ttk.Label(coord_frame, text="Yaw (deg)").grid(row=5, column=0, sticky="w", padx=4, pady=4)

        self.x_var = tk.StringVar(value="0.45")
        self.y_var = tk.StringVar(value="0.00")
        self.z_var = tk.StringVar(value="0.55")
        self.roll_var = tk.StringVar(value="0.0")
        self.pitch_var = tk.StringVar(value="0.0")
        self.yaw_var = tk.StringVar(value="0.0")
        self.t_var = tk.StringVar(value="3.0")
        self.use_orientation_var = tk.BooleanVar(value=False)
        self.orientation_mode_var = tk.StringVar(value="World")

        ttk.Entry(coord_frame, textvariable=self.x_var, width=16).grid(row=0, column=1, padx=4, pady=4)
        ttk.Entry(coord_frame, textvariable=self.y_var, width=16).grid(row=1, column=1, padx=4, pady=4)
        ttk.Entry(coord_frame, textvariable=self.z_var, width=16).grid(row=2, column=1, padx=4, pady=4)
        ttk.Entry(coord_frame, textvariable=self.roll_var, width=16).grid(row=3, column=1, padx=4, pady=4)
        ttk.Entry(coord_frame, textvariable=self.pitch_var, width=16).grid(row=4, column=1, padx=4, pady=4)
        ttk.Entry(coord_frame, textvariable=self.yaw_var, width=16).grid(row=5, column=1, padx=4, pady=4)

        ttk.Checkbutton(
            coord_frame,
            text="Use orientation",
            variable=self.use_orientation_var
        ).grid(row=6, column=0, columnspan=2, sticky="w", padx=4, pady=4)

        ttk.Label(coord_frame, text="Orientation reference").grid(row=7, column=0, sticky="w", padx=4, pady=4)
        ttk.Combobox(
            coord_frame,
            textvariable=self.orientation_mode_var,
            values=["World", "Hand-aligned"],
            state="readonly",
            width=14
        ).grid(row=7, column=1, padx=4, pady=4)

        ttk.Label(coord_frame, text="Time (sec)").grid(row=8, column=0, sticky="w", padx=4, pady=4)
        ttk.Entry(coord_frame, textvariable=self.t_var, width=16).grid(row=8, column=1, padx=4, pady=4)

        ttk.Button(
            coord_frame,
            text="Send coordinates",
            command=self.send_coordinates
        ).grid(row=9, column=0, columnspan=2, sticky="ew", pady=(10, 0))

        preset_frame = ttk.LabelFrame(main, text="Presets", padding=10)
        preset_frame.pack(fill="x", pady=10)

        ttk.Label(preset_frame, text="Preset").grid(row=0, column=0, sticky="w", padx=4, pady=4)
        ttk.Label(preset_frame, text="Time (sec)").grid(row=1, column=0, sticky="w", padx=4, pady=4)

        self.preset_var = tk.StringVar(value="Home")
        self.preset_time_var = tk.StringVar(value="3.0")

        ttk.Combobox(
            preset_frame,
            textvariable=self.preset_var,
            values=list(PRESETS.keys()),
            state="readonly",
            width=18
        ).grid(row=0, column=1, padx=4, pady=4)

        ttk.Entry(
            preset_frame,
            textvariable=self.preset_time_var,
            width=16
        ).grid(row=1, column=1, padx=4, pady=4)

        ttk.Button(
            preset_frame,
            text="Run preset",
            command=self.run_preset
        ).grid(row=2, column=0, columnspan=2, sticky="ew", pady=(10, 0))

        ttk.Button(
            main,
            text="ABORT / HOLD POSITION",
            command=self.abort_motion
        ).pack(fill="x", pady=(8, 0))

        info = (
            "hello world"
        )
        ttk.Label(main, text=info, justify="left").pack(anchor="w", pady=10)

        ttk.Button(main, text="Exit", command=self.root.destroy).pack(fill="x", pady=(8, 0))

    def _parse_float(self, value: str, field_name: str) -> float:
        try:
            return float(value)
        except ValueError:
            raise ValueError(f"{field_name} must be a number.")

    def send_coordinates(self):
        try:
            x = self._parse_float(self.x_var.get(), "X")
            y = self._parse_float(self.y_var.get(), "Y")
            z = self._parse_float(self.z_var.get(), "Z")
            roll = self._parse_float(self.roll_var.get(), "Roll")
            pitch = self._parse_float(self.pitch_var.get(), "Pitch")
            yaw = self._parse_float(self.yaw_var.get(), "Yaw")
            t = self._parse_float(self.t_var.get(), "Time")

            if t <= 0.0:
                raise ValueError("Time must be greater than 0.")
        except ValueError as e:
            messagebox.showerror("Input error", str(e))
            return

        self.node.publish_coordinates(
            x, y, z, roll, pitch, yaw, t,
            self.use_orientation_var.get(),
            self.orientation_mode_var.get()
        )

    def run_preset(self):
        try:
            preset_name = self.preset_var.get()
            t = self._parse_float(self.preset_time_var.get(), "Preset time")
            if t <= 0.0:
                raise ValueError("Preset time must be greater than 0.")
        except ValueError as e:
            messagebox.showerror("Input error", str(e))
            return

        if preset_name == "Wave":
            self.node.start_wave()
            messagebox.showinfo(
                "Preset",
                "Wave motion started. It will continue until another command is sent."
            )
            return

        self.node.stop_wave()

        preset = PRESETS[preset_name]

        if len(preset) == 4:
            x, y, z, _default_t = preset
            self.node.publish_coordinates(
                x, y, z, 0.0, 0.0, 0.0, t,
                False,
                self.orientation_mode_var.get()
            )
        elif len(preset) == 7:
            x, y, z, roll, pitch, yaw, _default_t = preset
            self.node.publish_coordinates(
                x, y, z, roll, pitch, yaw, t,
                True,
                self.orientation_mode_var.get()
            )
        else:
            messagebox.showerror("Preset error", f"Preset '{preset_name}' has invalid format.")
            return

    def abort_motion(self):
        ok = self.node.abort_motion()
        if ok:
            messagebox.showwarning("Abort", "Abort command sent. Robot should hold current position.")
        else:
            messagebox.showerror("Abort", "No joint state available yet. Abort could not be sent.")

    def handle_status(self, text: str):
        self.root.after(0, lambda: self._handle_status_ui(text))

    def _handle_status_ui(self, text: str):
        if text.startswith("ERROR:"):
            messagebox.showerror("Planner / IK error", text)
        elif text.startswith("OK:"):
            messagebox.showinfo("Planner / IK", text)

    def run(self):
        self.root.mainloop()


def ros_spin(node):
    rclpy.spin(node)


def main():
    rclpy.init()
    node = PumaGuiNode()

    thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    thread.start()

    gui = PumaControlGUI(node)
    try:
        gui.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
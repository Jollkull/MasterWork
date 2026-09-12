#!/usr/bin/env python3
import math
import threading
import time
from dataclasses import dataclass
from typing import List, Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from trajectory_msgs.msg import JointTrajectory

try:
    import serial  # pyserial
except ImportError:
    serial = None


@dataclass
class TrajPointCmd:
    servo_vals: List[int]  # 6 ints 0..180
    dt_ms: int             # delay until applying this point


def clamp_int(x: int, lo: int, hi: int) -> int:
    return lo if x < lo else hi if x > hi else x


class ArduinoServoBridge(Node):
    """
    Subscribes to JointTrajectory (same as gazebo joint_trajectory_controller),
    maps joint angles (rad) -> servo degrees [0..180],
    sends to Arduino: "G j1 j2 j3 j4 j5 j6 t(ms)\\n"
    """

    def __init__(self):
        super().__init__('arduino_servo_bridge')

        # ---- parameters ----
        self.declare_parameter('port', '/tmp/ttyARDUINO')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('topic_in', '/joint_trajectory_controller/joint_trajectory')
        self.declare_parameter('joint_order', ['j1', 'j2', 'j3', 'j4', 'j5', 'j6'])

        # Mapping: servo_deg = round( rad * 180/pi + offset_deg ) * scale  (scale default 1)
        # Default offset_deg = +90 (so -90..+90 maps to 0..180)
        self.declare_parameter('offset_deg', [90.0, 90.0, 90.0, 90.0, 90.0, 90.0])
        self.declare_parameter('scale',      [1.0,  1.0,  1.0,  1.0,  1.0,  1.0])

        self.declare_parameter('servo_min', 0)
        self.declare_parameter('servo_max', 180)

        # If trajectory point dt is 0 (or negative), use this fallback
        self.declare_parameter('default_dt_ms', 20)

        # If true: write to serial immediately on receive (only last point)
        # If false (default): play full trajectory with timing
        self.declare_parameter('play_full_trajectory', True)

        # ---- serial ----
        if serial is None:
            raise RuntimeError("pyserial not installed. Install: apt install python3-serial")

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)

        self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        self.get_logger().info(f"Opened serial: {self.port} @ {self.baud}")

        # ---- worker thread + queue ----
        self._queue: List[TrajPointCmd] = []
        self._queue_lock = threading.Lock()
        self._queue_event = threading.Event()
        self._stop = False

        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        topic_in = self.get_parameter('topic_in').value
        self.sub = self.create_subscription(JointTrajectory, topic_in, self.on_traj, qos)
        self.get_logger().info(f"Subscribed: {topic_in}")

        self.get_logger().info("Bridge ready. Send JointTrajectory and I will stream G-lines to Arduino.")

    def destroy_node(self):
        self._stop = True
        self._queue_event.set()
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()

    def on_traj(self, msg: JointTrajectory):
        joint_order = list(self.get_parameter('joint_order').value)
        offsets = list(self.get_parameter('offset_deg').value)
        scales = list(self.get_parameter('scale').value)
        servo_min = int(self.get_parameter('servo_min').value)
        servo_max = int(self.get_parameter('servo_max').value)
        default_dt_ms = int(self.get_parameter('default_dt_ms').value)
        play_full = bool(self.get_parameter('play_full_trajectory').value)

        if len(joint_order) != 6:
            self.get_logger().error("joint_order must have exactly 6 joint names.")
            return
        if len(offsets) != 6 or len(scales) != 6:
            self.get_logger().error("offset_deg and scale must have exactly 6 values each.")
            return

        # Map msg.joint_names -> indices in msg.points[].positions
        name_to_index: Dict[str, int] = {n: i for i, n in enumerate(msg.joint_names)}

        missing = [jn for jn in joint_order if jn not in name_to_index]
        if missing:
            self.get_logger().warn(f"Trajectory missing joints: {missing}. "
                                   f"Incoming joints: {list(msg.joint_names)}")
            return

        if not msg.points:
            self.get_logger().warn("Empty trajectory points.")
            return

        # Build commands from points
        cmds: List[TrajPointCmd] = []
        prev_t = 0.0

        points_to_play = msg.points if play_full else [msg.points[-1]]

        for p in points_to_play:
            # absolute time_from_start in seconds
            t_abs = float(p.time_from_start.sec) + float(p.time_from_start.nanosec) * 1e-9
            dt = t_abs - prev_t
            prev_t = t_abs

            dt_ms = int(round(dt * 1000.0))
            if dt_ms <= 0:
                dt_ms = default_dt_ms

            servo_vals: List[int] = []
            for k, jn in enumerate(joint_order):
                idx = name_to_index[jn]
                rad = float(p.positions[idx])

                deg = rad * 180.0 / math.pi
                mapped = (deg + float(offsets[k])) * float(scales[k])

                servo = clamp_int(int(round(mapped)), servo_min, servo_max)
                servo_vals.append(servo)

            cmds.append(TrajPointCmd(servo_vals=servo_vals, dt_ms=dt_ms))

        # Replace queue with new trajectory (drop previous)
        with self._queue_lock:
            self._queue = cmds
        self._queue_event.set()

        self.get_logger().info(
            f"Queued {len(cmds)} point(s). First: {cmds[0].servo_vals} dt={cmds[0].dt_ms}ms"
        )

    def _worker_loop(self):
        while not self._stop:
            self._queue_event.wait(timeout=0.5)
            if self._stop:
                break

            # Grab current queue snapshot
            with self._queue_lock:
                cmds = list(self._queue)
                self._queue = []
            self._queue_event.clear()

            for cmd in cmds:
                if self._stop:
                    break

                line = f"G {cmd.servo_vals[0]} {cmd.servo_vals[1]} {cmd.servo_vals[2]} " \
                       f"{cmd.servo_vals[3]} {cmd.servo_vals[4]} {cmd.servo_vals[5]} {cmd.dt_ms}\n"
                try:
                    self.ser.write(line.encode('ascii'))
                except Exception as e:
                    self.get_logger().error(f"Serial write failed: {e}")
                    break

                time.sleep(cmd.dt_ms / 1000.0)


def main():
    rclpy.init()
    node = ArduinoServoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

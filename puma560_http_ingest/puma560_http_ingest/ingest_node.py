#!/usr/bin/env python3
"""
ROS2 Foxy-friendly ingestion node (Pub/Sub):
- no AsyncIOExecutor
- batching via debounce (threading.Timer)
- spam-filter for controller state
- publishes batches to Google Pub/Sub topic
- payload compatible with Pub/Sub -> BigQuery subscription "Use table schema"
  expected table columns (recommended):
    robot_id STRING
    sent_time_ns INT64
    batch_size INT64
    payload_json STRING
  + (optional) metadata columns if you enabled "Write metadata"
"""

import base64
import gzip
import json
import math
import os
import threading
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import JointTrajectoryControllerState

from google.cloud import pubsub_v1

import traceback



def ros_time_to_dict(stamp) -> Dict[str, int]:
    return {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)}


def finite_list(xs) -> List[Optional[float]]:
    out: List[Optional[float]] = []
    for x in xs:
        fx = float(x)
        out.append(fx if math.isfinite(fx) else None)
    return out


def joint_trajectory_to_dict(msg: JointTrajectory) -> Dict[str, Any]:
    return {
        "header": {"stamp": ros_time_to_dict(msg.header.stamp), "frame_id": msg.header.frame_id},
        "joint_names": list(msg.joint_names),
        "points": [
            {
                "positions": finite_list(p.positions),
                "velocities": finite_list(p.velocities),
                "accelerations": finite_list(p.accelerations),
                "effort": finite_list(p.effort),
                "time_from_start": ros_time_to_dict(p.time_from_start),
            }
            for p in msg.points
        ],
    }


def controller_state_to_dict(msg: JointTrajectoryControllerState) -> Dict[str, Any]:
    def point_to_dict(pt):
        return {
            "positions": finite_list(pt.positions),
            "velocities": finite_list(pt.velocities),
            "accelerations": finite_list(pt.accelerations),
            "effort": finite_list(pt.effort),
            "time_from_start": ros_time_to_dict(pt.time_from_start),
        }

    return {
        "header": {"stamp": ros_time_to_dict(msg.header.stamp), "frame_id": msg.header.frame_id},
        "joint_names": list(msg.joint_names),
        "desired": point_to_dict(msg.desired),
        "actual": point_to_dict(msg.actual),
        "error": point_to_dict(msg.error),
    }


def float64_multiarray_to_dict(msg: Float64MultiArray) -> Dict[str, Any]:
    return {"data": finite_list(msg.data)}


def max_abs_diff(a: List[Optional[float]], b: List[Optional[float]]) -> float:
    if not a and not b:
        return 0.0
    if len(a) != len(b):
        return float("inf")
    m = 0.0
    for av, bv in zip(a, b):
        if av is None or bv is None:
            continue
        m = max(m, abs(float(av) - float(bv)))
    return m


@dataclass
class BufferedMsg:
    topic: str
    msg_type: str
    received_time_ns: int
    payload: Dict[str, Any]


class Puma560PubSubIngest(Node):
    def __init__(self):
        super().__init__("puma560_pubsub_ingest")

        # -------- Parameters --------
        self.declare_parameter("robot_id", "puma560")
        self.declare_parameter("debounce_sec", 2.0)
        self.declare_parameter("state_epsilon", 1e-4)
        self.declare_parameter("max_batch_size", 5000)
        self.declare_parameter("log_payload_bytes", 0)

        # Pub/Sub
        # You can pass full topic path: projects/<project>/topics/<topic>
        # Or provide project_id + topic_id.
        self.declare_parameter("gcp_project_id", "")
        self.declare_parameter("pubsub_topic_id", "ros-batches")
        self.declare_parameter("pubsub_topic_path", "")  # optional override
        self.declare_parameter("enable_ordering", True)
        self.declare_parameter("ordering_key_mode", "robot_id")  # robot_id | none
        self.declare_parameter("publish_timeout_sec", 5.0)

        # Optional: compress the batch JSON inside payload_json (base64(gzip(json))).
        # Only enable if you ALSO handle decompression later in BigQuery pipeline (Silver step).
        self.declare_parameter("payload_compress_gzip", False)
        self.declare_parameter("payload_gzip_level", 6)
        self.declare_parameter("payload_gzip_min_bytes", 2048)

        self.robot_id: str = str(self.get_parameter("robot_id").value)
        self.debounce_sec: float = float(self.get_parameter("debounce_sec").value)
        self.state_epsilon: float = float(self.get_parameter("state_epsilon").value)
        self.max_batch_size: int = int(self.get_parameter("max_batch_size").value)
        self.log_payload_bytes: int = int(self.get_parameter("log_payload_bytes").value)

        self.gcp_project_id: str = str(self.get_parameter("gcp_project_id").value)
        self.pubsub_topic_id: str = str(self.get_parameter("pubsub_topic_id").value)
        self.pubsub_topic_path: str = str(self.get_parameter("pubsub_topic_path").value)

        self.enable_ordering: bool = bool(self.get_parameter("enable_ordering").value)
        self.ordering_key_mode: str = str(self.get_parameter("ordering_key_mode").value)
        self.publish_timeout_sec: float = float(self.get_parameter("publish_timeout_sec").value)

        self.payload_compress_gzip: bool = bool(self.get_parameter("payload_compress_gzip").value)
        self.payload_gzip_level: int = int(self.get_parameter("payload_gzip_level").value)
        self.payload_gzip_min_bytes: int = int(self.get_parameter("payload_gzip_min_bytes").value)

        # If project_id not provided via param, try env var
        if not self.gcp_project_id:
            self.gcp_project_id = os.getenv("GOOGLE_CLOUD_PROJECT", "") or os.getenv("GCP_PROJECT", "")

        # Resolve topic path
        if self.pubsub_topic_path:
            topic_path = self.pubsub_topic_path
        else:
            if not self.gcp_project_id:
                raise RuntimeError(
                    "gcp_project_id is empty. Set ROS param 'gcp_project_id' "
                    "or env GOOGLE_CLOUD_PROJECT, or set 'pubsub_topic_path'."
                )
            topic_path = f"projects/{self.gcp_project_id}/topics/{self.pubsub_topic_id}"
        self._topic_path = topic_path

        # -------- QoS --------
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # -------- Buffer + locks --------
        self._buf_lock = threading.Lock()
        self._buffer: List[BufferedMsg] = []
        self._debounce_timer: Optional[threading.Timer] = None
        self._last_state_actual: Optional[Dict[str, Any]] = None

        # -------- Pub/Sub publisher --------
        publisher_options = pubsub_v1.types.PublisherOptions(enable_message_ordering=self.enable_ordering)
        self._publisher = pubsub_v1.PublisherClient(publisher_options=publisher_options)

        # -------- Subscriptions --------
        self.create_subscription(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            self._on_joint_trajectory,
            qos,
        )
        self.create_subscription(
            JointTrajectoryControllerState,
            "/joint_trajectory_controller/state",
            self._on_controller_state,
            qos,
        )
        self.create_subscription(
            Float64MultiArray,
            "/puma560/target_xyz_period",
            self._on_target_xyz_period,
            qos,
        )

        self.get_logger().info(
            f"Pub/Sub ingest started. topic={self._topic_path}, debounce={self.debounce_sec}s, "
            f"state_epsilon={self.state_epsilon}, ordering={self.enable_ordering}"
        )

    # ---------- Buffering / debounce ----------

    def _schedule_flush(self):
        if self._debounce_timer is not None:
            self._debounce_timer.cancel()
        self._debounce_timer = threading.Timer(self.debounce_sec, self._flush_from_timer_thread)
        self._debounce_timer.daemon = True
        self._debounce_timer.start()

    def _buffer_msg(self, topic: str, msg_type: str, payload: Dict[str, Any]):
        now_ns = int(self.get_clock().now().nanoseconds)
        with self._buf_lock:
            self._buffer.append(BufferedMsg(topic, msg_type, now_ns, payload))
            if len(self._buffer) > self.max_batch_size:
                drop = len(self._buffer) - self.max_batch_size
                self._buffer = self._buffer[drop:]
                self.get_logger().warn(f"Buffer exceeded max_batch_size; dropped {drop} oldest messages.")
        self._schedule_flush()

    def _flush_from_timer_thread(self):
        # Timer thread -> do publish synchronously (PublisherClient is thread-safe)
        try:
            self._flush_and_publish()
        except Exception as e:
            self.get_logger().error(f"Flush/publish exception: {e}")

    # ---------- ROS callbacks ----------

    def _on_joint_trajectory(self, msg: JointTrajectory):
        self._buffer_msg(
            "/joint_trajectory_controller/joint_trajectory",
            "trajectory_msgs/JointTrajectory",
            joint_trajectory_to_dict(msg),
        )

    def _on_target_xyz_period(self, msg: Float64MultiArray):
        self._buffer_msg(
            "/puma560/target_xyz_period",
            "std_msgs/Float64MultiArray",
            float64_multiarray_to_dict(msg),
        )

    def _on_controller_state(self, msg: JointTrajectoryControllerState):
        d = controller_state_to_dict(msg)
        actual = d.get("actual", {})

        with self._buf_lock:
            if self._last_state_actual is not None:
                diffs = [
                    max_abs_diff(actual.get("positions", []), self._last_state_actual.get("positions", [])),
                    max_abs_diff(actual.get("velocities", []), self._last_state_actual.get("velocities", [])),
                    max_abs_diff(actual.get("accelerations", []), self._last_state_actual.get("accelerations", [])),
                    max_abs_diff(actual.get("effort", []), self._last_state_actual.get("effort", [])),
                ]
                if max(diffs) < self.state_epsilon:
                    return
            self._last_state_actual = actual

        self._buffer_msg(
            "/joint_trajectory_controller/state",
            "control_msgs/JointTrajectoryControllerState",
            d,
        )

    # ---------- Pub/Sub publish ----------

    def _maybe_compress_payload_json(self, payload_json: str) -> str:
        """
        If enabled and big enough: payload_json becomes:
          {"encoding":"gzip+base64","data":"..."}
        returned as a STRING (still fits BigQuery STRING column).
        """
        if not self.payload_compress_gzip:
            return payload_json

        raw = payload_json.encode("utf-8")
        if len(raw) < self.payload_gzip_min_bytes:
            return payload_json

        try:
            gz = gzip.compress(raw, compresslevel=self.payload_gzip_level)
            b64 = base64.b64encode(gz).decode("ascii")
            wrapper = {"encoding": "gzip+base64", "data": b64}
            return json.dumps(wrapper, separators=(",", ":"))
        except Exception as e:
            self.get_logger().warn(f"payload gzip failed, sending raw payload_json: {e}")
            return payload_json

    def _flush_and_publish(self):
        # Take snapshot of buffer
        with self._buf_lock:
            if not self._buffer:
                return
            batch = self._buffer
            self._buffer = []

        sent_time_ns = int(self.get_clock().now().nanoseconds)

        batch_body = {
            "robot_id": self.robot_id,
            "batch_size": len(batch),
            "sent_time_ns": sent_time_ns,
            "items": [
                {
                    "topic": it.topic,
                    "type": it.msg_type,
                    "received_time_ns": it.received_time_ns,
                    "payload": it.payload,
                }
                for it in batch
            ],
        }

        # This is what we store in BigQuery column payload_json (STRING)
        payload_json = json.dumps(batch_body, ensure_ascii=False, separators=(",", ":"))
        payload_json = self._maybe_compress_payload_json(payload_json)

        # This is the Pub/Sub message JSON that must match BigQuery table schema
        row = {
            "robot_id": self.robot_id,
            "sent_time_ns": sent_time_ns,
            "batch_size": len(batch),
            "payload_json": payload_json,
        }

        data = json.dumps(row, ensure_ascii=False, separators=(",", ":")).encode("utf-8")

        if self.log_payload_bytes and self.log_payload_bytes > 0:
            s = data.decode("utf-8", errors="replace")
            self.get_logger().info("Pub/Sub message (truncated): " + s[: self.log_payload_bytes])

        ordering_key = ""
        if self.enable_ordering and self.ordering_key_mode == "robot_id":
            ordering_key = self.robot_id

        try:
            future = self._publisher.publish(
                self._topic_path,
                data=data,
                ordering_key=ordering_key if ordering_key else None,
            )

            def _done(fut):
                try:
                    msg_id = fut.result()
                    self.get_logger().info(
                        f"Published batch to Pub/Sub: msgs={len(batch)}, message_id={msg_id}, sent_time_ns={sent_time_ns}"
                    )
                except Exception as e:
                    self.get_logger().error(
                        f"Pub/Sub publish callback failed: {type(e).__name__}: {e}\n{traceback.format_exc()}"
                    )
                    # re-queue
                    with self._buf_lock:
                        self._buffer = batch + self._buffer
                    # if ordering enabled, resume
                    try:
                        if self.enable_ordering and self.ordering_key_mode == "robot_id":
                            self._publisher.resume_publish(self._topic_path, self.robot_id)
                    except Exception:
                        pass

            future.add_done_callback(_done)

        except Exception as e:
            self.get_logger().error(f"Pub/Sub publish failed: {e}")
            # Re-queue on failure
            with self._buf_lock:
                self._buffer = batch + self._buffer
            # If ordering enabled, publisher might get "paused" after an error.
            # Resume to avoid deadlock on subsequent publishes.
            try:
                if self.enable_ordering:
                    self._publisher.resume_publish(self._topic_path, self.robot_id)
            except Exception:
                pass

    def destroy_node(self):
        try:
            if self._debounce_timer is not None:
                self._debounce_timer.cancel()
        except Exception:
            pass

        try:
            # Flush remaining messages once (best effort)
            self._flush_and_publish()
        except Exception:
            pass

        try:
            self._publisher.stop()  # flush background threads
        except Exception:
            pass

        super().destroy_node()


def main():
    rclpy.init()
    node = Puma560PubSubIngest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

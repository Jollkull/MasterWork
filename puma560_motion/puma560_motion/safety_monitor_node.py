import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import tf2_ros
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64, String
from control_msgs.msg import SpeedScalingFactor
from visualization_msgs.msg import Marker, MarkerArray


class SafetyMonitorNode(Node):
    def __init__(self):
        super().__init__('safety_monitor_node')

        # --- Параметры (можно переопределить через launch/CLI) ---
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter(
            'link_names',
            ['link4', 'link5', 'link6', 'link7', 'tool0'],
        )
        self.declare_parameter('distance_warn', 0.6)   # порог 100% -> 30%
        self.declare_parameter('distance_stop', 0.3)   # порог 30% -> 0%
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('human_position_topic', '/puma/human_position')
        self.declare_parameter('status_text_frame', 'tool0')
        self.declare_parameter('status_text_z_offset', 0.3)

        self.world_frame = self.get_parameter('world_frame').value
        self.link_names = self.get_parameter('link_names').value
        self.distance_warn = self.get_parameter('distance_warn').value
        self.distance_stop = self.get_parameter('distance_stop').value
        rate_hz = self.get_parameter('rate_hz').value
        human_topic = self.get_parameter('human_position_topic').value

        # --- TF ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- Состояние ---
        self.human_position = None  # (x, y, z) в world_frame
        self.current_mode = 'SAFE'  # SAFE / SLOW / STOP

        # --- Подписки/публикации ---
        self.create_subscription(
            PointStamped, human_topic, self.human_position_callback, 10
        )
        self.status_pub = self.create_publisher(String, '/puma/safety_status', 10)
        scaling_qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        depth=1,
    )
        self.scaling_pub = self.create_publisher(
            SpeedScalingFactor, '/joint_trajectory_controller/speed_scaling_input', scaling_qos
)
        self.min_distance_pub = self.create_publisher(
            Float64, '/puma/min_distance', 10
        )
        self.status_marker_pub = self.create_publisher(
            Marker, '/puma/status_marker', 10
        )
        self.link_markers_pub = self.create_publisher(
            MarkerArray, '/puma/link_safety_markers', 10
        )

        self.timer = self.create_timer(1.0 / rate_hz, self.tick)

        self.get_logger().info(
            f'safety_monitor_node started. warn={self.distance_warn}m '
            f'stop={self.distance_stop}m links={self.link_names}'
        )

    def human_position_callback(self, msg: PointStamped):
        self.human_position = (msg.point.x, msg.point.y, msg.point.z)

    def get_link_position(self, link_name: str):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame, link_name, Time()
            )
            t = tf.transform.translation
            return (t.x, t.y, t.z)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return None

    @staticmethod
    def distance(a, b):
        return math.sqrt(
            (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2
        )

    def tick(self):
        link_positions = {}
        for link in self.link_names:
            pos = self.get_link_position(link)
            if pos is not None:
                link_positions[link] = pos

        self.publish_link_safety_markers(link_positions)

        if self.human_position is None:
            self.publish_mode('SAFE', 1.0, None)
            return

        min_dist = None
        for pos in link_positions.values():
            d = self.distance(pos, self.human_position)
            if min_dist is None or d < min_dist:
                min_dist = d

        if min_dist is None:
            return

        if min_dist <= self.distance_stop:
            mode, scale = 'STOP', 0.0
        elif min_dist <= self.distance_warn:
            mode, scale = 'SLOW', 0.3
        else:
            mode, scale = 'SAFE', 1.0

        self.publish_mode(mode, scale, min_dist)

    def publish_mode(self, mode: str, scale: float, min_dist):
        if mode != self.current_mode:
            self.get_logger().info(
                f'Safety mode changed: {self.current_mode} -> {mode} '
                f'(min_dist={min_dist})'
            )
            self.current_mode = mode

        self.status_pub.publish(String(data=mode))

        scaling_msg = SpeedScalingFactor()
        scaling_msg.factor = scale
        self.scaling_pub.publish(scaling_msg)

        if min_dist is not None:
            self.min_distance_pub.publish(Float64(data=float(min_dist)))

        self.publish_status_marker(mode, min_dist)

    def publish_status_marker(self, mode: str, min_dist):
        text_frame = self.get_parameter('status_text_frame').value
        z_offset = self.get_parameter('status_text_z_offset').value

        pos = self.get_link_position(text_frame)
        if pos is None:
            return

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.world_frame
        marker.ns = 'safety_status'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = pos[0]
        marker.pose.position.y = pos[1]
        marker.pose.position.z = pos[2] + z_offset
        marker.pose.orientation.w = 1.0

        marker.scale.z = 0.12

        if mode == 'SAFE':
            marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.0
            marker.text = 'SAFE'
        elif mode == 'SLOW':
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.85, 0.0
            marker.text = 'SLOW'
        else:
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0
            marker.text = 'STOP'

        if min_dist is not None:
            marker.text += f'\n{min_dist:.2f}m'

        marker.color.a = 1.0
        marker.lifetime.sec = 0

        self.status_marker_pub.publish(marker)

    def publish_link_safety_markers(self, link_positions: dict):
        array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for idx, (link_name, pos) in enumerate(link_positions.items()):
            # --- внешняя сфера: зона WARN ---
            warn_marker = Marker()
            warn_marker.header.stamp = stamp
            warn_marker.header.frame_id = self.world_frame
            warn_marker.ns = 'link_safety_warn'
            warn_marker.id = idx
            warn_marker.type = Marker.SPHERE
            warn_marker.action = Marker.ADD
            warn_marker.pose.position.x = pos[0]
            warn_marker.pose.position.y = pos[1]
            warn_marker.pose.position.z = pos[2]
            warn_marker.pose.orientation.w = 1.0
            d_warn = self.distance_warn * 2.0
            warn_marker.scale.x = d_warn
            warn_marker.scale.y = d_warn
            warn_marker.scale.z = d_warn
            warn_marker.color.r = 1.0
            warn_marker.color.g = 0.85
            warn_marker.color.b = 0.0
            warn_marker.color.a = 0.12
            warn_marker.lifetime.sec = 0
            array.markers.append(warn_marker)

            # --- внутренняя сфера: зона STOP ---
            stop_marker = Marker()
            stop_marker.header.stamp = stamp
            stop_marker.header.frame_id = self.world_frame
            stop_marker.ns = 'link_safety_stop'
            stop_marker.id = idx
            stop_marker.type = Marker.SPHERE
            stop_marker.action = Marker.ADD
            stop_marker.pose.position.x = pos[0]
            stop_marker.pose.position.y = pos[1]
            stop_marker.pose.position.z = pos[2]
            stop_marker.pose.orientation.w = 1.0
            d_stop = self.distance_stop * 2.0
            stop_marker.scale.x = d_stop
            stop_marker.scale.y = d_stop
            stop_marker.scale.z = d_stop
            stop_marker.color.r = 1.0
            stop_marker.color.g = 0.0
            stop_marker.color.b = 0.0
            stop_marker.color.a = 0.18
            stop_marker.lifetime.sec = 0
            array.markers.append(stop_marker)

        self.link_markers_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
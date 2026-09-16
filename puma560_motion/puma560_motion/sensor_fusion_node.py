import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker


class SensorFusionNode(Node):
    """
    Объединение оценок позиции человека от лидара и камеры глубины.

    Правила:
      - оба источника свежие -> взвешенное среднее
      - свежий только один -> используется он (компенсация окклюзии)
      - оба устарели -> позиция не публикуется
    """

    def __init__(self):
        super().__init__('sensor_fusion_node')

        self.declare_parameter('lidar_topic', '/puma/human_position_lidar')
        self.declare_parameter('camera_topic', '/puma/human_position_camera')
        self.declare_parameter('output_topic', '/puma/human_position')
        self.declare_parameter('world_frame', 'world')
        # Данные старше этого возраста считаются неактуальными, с
        self.declare_parameter('sensor_timeout', 0.6)
        # Веса источников при усреднении
        self.declare_parameter('lidar_weight', 0.4)
        self.declare_parameter('camera_weight', 0.6)
        # Если источники расходятся больше этого - доверяем камере, с
        self.declare_parameter('disagreement_threshold', 0.8)
        self.declare_parameter('rate_hz', 10.0)
        # Сенсоры измеряют разные точки по высоте одного объекта,
        # поэтому Z берётся как центр тела человека, а не усредняется
        self.declare_parameter('human_center_height', 0.875)

        self.world_frame = self.get_parameter('world_frame').value

        self.lidar_point = None
        self.lidar_time = None
        self.camera_point = None
        self.camera_time = None

        self.last_mode = None

        self.create_subscription(
            PointStamped, self.get_parameter('lidar_topic').value,
            self.lidar_callback, 10,
        )
        self.create_subscription(
            PointStamped, self.get_parameter('camera_topic').value,
            self.camera_callback, 10,
        )

        self.position_pub = self.create_publisher(
            PointStamped, self.get_parameter('output_topic').value, 10
        )
        self.source_pub = self.create_publisher(
            String, '/puma/fusion_source', 10
        )
        self.marker_pub = self.create_publisher(
            Marker, '/puma/fused_human_marker', 10
        )

        rate = self.get_parameter('rate_hz').value
        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            'sensor_fusion_node started. '
            f"{self.get_parameter('lidar_topic').value} + "
            f"{self.get_parameter('camera_topic').value} -> "
            f"{self.get_parameter('output_topic').value}"
        )

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def lidar_callback(self, msg: PointStamped):
        self.lidar_point = msg
        self.lidar_time = self.now_sec()

    def camera_callback(self, msg: PointStamped):
        self.camera_point = msg
        self.camera_time = self.now_sec()

    def is_fresh(self, stamp):
        if stamp is None:
            return False
        timeout = self.get_parameter('sensor_timeout').value
        return (self.now_sec() - stamp) <= timeout

    def tick(self):
        lidar_ok = self.is_fresh(self.lidar_time)
        camera_ok = self.is_fresh(self.camera_time)

        if lidar_ok and camera_ok:
            point, mode = self.combine()
        elif camera_ok:
            point, mode = self.camera_point, 'CAMERA_ONLY'
        elif lidar_ok:
            point, mode = self.lidar_point, 'LIDAR_ONLY'
        else:
            self.report_mode('NO_DATA')
            return

        if point is None:
            return

        out = PointStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.world_frame
        out.point.x = point.point.x
        out.point.y = point.point.y
        out.point.z = self.get_parameter('human_center_height').value

        self.position_pub.publish(out)
        self.source_pub.publish(String(data=mode))
        self.publish_marker(out, mode)
        self.report_mode(mode)

    def combine(self):
        lp = self.lidar_point.point
        cp = self.camera_point.point

        gap = math.hypot(lp.x - cp.x, lp.y - cp.y)
        threshold = self.get_parameter('disagreement_threshold').value

        # Сильное расхождение - вероятно, один сенсор видит не тот объект.
        # Доверяем камере: она даёт объёмную картину, лидар - только срез.
        if gap > threshold:
            return self.camera_point, 'DISAGREEMENT_CAMERA'

        wl = self.get_parameter('lidar_weight').value
        wc = self.get_parameter('camera_weight').value
        total = wl + wc

        fused = PointStamped()
        fused.point.x = (lp.x * wl + cp.x * wc) / total
        fused.point.y = (lp.y * wl + cp.y * wc) / total
        fused.point.z = (lp.z * wl + cp.z * wc) / total
        return fused, 'FUSED'

    def publish_marker(self, point: PointStamped, mode: str):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.world_frame
        marker.ns = 'fused_human'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = point.point.x
        marker.pose.position.y = point.point.y
        marker.pose.position.z = 0.875
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 1.75

        if mode == 'FUSED':
            marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.3
        elif mode.startswith('CAMERA'):
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.5, 0.0
        elif mode.startswith('DISAGREEMENT'):
            marker.color.r, marker.color.g, marker.color.b = 1.0, 1.0, 0.0
        else:
            marker.color.r, marker.color.g, marker.color.b = 0.0, 0.5, 1.0

        marker.color.a = 0.45
        marker.lifetime.sec = 1
        self.marker_pub.publish(marker)

    def report_mode(self, mode):
        if mode != self.last_mode:
            self.get_logger().info(f'Fusion source: {mode}')
            self.last_mode = mode


def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

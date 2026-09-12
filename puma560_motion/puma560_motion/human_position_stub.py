import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker


class HumanPositionStub(Node):
    """
    Временная заглушка вместо реального сенсора (лидар/камера).
    Публикует статичную/управляемую точку как позицию 'человека'
    и визуальный маркер (сферу) для отображения в RViz2.
    Меняйте параметры на лету:
      ros2 param set /human_position_stub x 0.3
    """

    def __init__(self):
        super().__init__('human_position_stub')

        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('x', 0.8)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.3)
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('marker_radius', 0.15)

        self.pub = self.create_publisher(
            PointStamped, '/puma/human_position', 10
        )
        self.marker_pub = self.create_publisher(
            Marker, '/puma/human_marker', 10
        )

        rate_hz = self.get_parameter('rate_hz').value
        self.timer = self.create_timer(1.0 / rate_hz, self.tick)
        self.get_logger().info(
            'human_position_stub publishing on /puma/human_position and /puma/human_marker'
        )

    def tick(self):
        frame = self.get_parameter('world_frame').value
        x = self.get_parameter('x').value
        y = self.get_parameter('y').value
        z = self.get_parameter('z').value
        stamp = self.get_clock().now().to_msg()

        # --- публикуем позицию (для safety_monitor_node) ---
        point_msg = PointStamped()
        point_msg.header.stamp = stamp
        point_msg.header.frame_id = frame
        point_msg.point.x = x
        point_msg.point.y = y
        point_msg.point.z = z
        self.pub.publish(point_msg)

        # --- публикуем маркер (для отображения в RViz2) ---
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame
        marker.ns = 'human'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0

        radius = self.get_parameter('marker_radius').value
        marker.scale.x = radius * 2.0
        marker.scale.y = radius * 2.0
        marker.scale.z = radius * 2.0

        # Цвет по умолчанию - жёлтый, полупрозрачный
        marker.color.r = 1.0
        marker.color.g = 0.9
        marker.color.b = 0.0
        marker.color.a = 0.8

        marker.lifetime.sec = 0  # 0 = живёт пока не обновится
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = HumanPositionStub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

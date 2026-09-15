import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import tf2_ros
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker


class LidarHumanDetector(Node):
    """
    Обнаруживает объект (человека) по данным лидара.
    """

    def __init__(self):
        super().__init__('lidar_human_detector')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('output_topic', '/puma/human_position')
        self.declare_parameter('world_frame', 'world')
        # Gazebo даёт своё имя фрейма (puma560/link1/safety_lidar),
        # которого нет в TF-дереве ROS2 - используем имя из URDF
        self.declare_parameter('lidar_frame', 'lidar_link')
        self.declare_parameter('cluster_distance_threshold', 0.15)
        self.declare_parameter('min_cluster_points', 3)
        self.declare_parameter('min_object_width', 0.10)
        self.declare_parameter('max_object_width', 1.20)
        self.declare_parameter('max_detection_range', 8.0)
        self.declare_parameter('publish_marker', True)
        self.declare_parameter('debug_clusters', True)
        # Всё ближе этой дистанции считается частью самого робота
        self.declare_parameter('min_detection_range', 0.3)
        # Звенья робота, которые нужно исключить из детекции
        self.declare_parameter(
            'robot_links',
            ['link1', 'link2', 'link3', 'link4', 'link5', 'link6', 'link7', 'tool0'],
        )
        # Кластер ближе этого расстояния к любому звену считается самим роботом, м
        self.declare_parameter('robot_exclusion_radius', 0.35)
        # Максимальный угловой зазор между кластерами для склейки, рад
        self.declare_parameter('merge_angle_gap', 0.35)
        # Максимальная разница дистанций для склейки, м
        self.declare_parameter('merge_range_gap', 0.25)

        self.world_frame = self.get_parameter('world_frame').value
        scan_topic = self.get_parameter('scan_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)
        self.position_pub = self.create_publisher(PointStamped, output_topic, 10)
        self.marker_pub = self.create_publisher(
            Marker, '/puma/detected_human_marker', 10
        )

        self.last_detection_logged = None

        self.get_logger().info(
            f'lidar_human_detector started. Listening on {scan_topic}, '
            f'publishing to {output_topic}'
        )

    def scan_callback(self, msg: LaserScan):
        clusters = self.extract_clusters(msg)
        if not clusters:
            if self.last_detection_logged is not False:
                self.get_logger().info('No object detected in scan')
                self.last_detection_logged = False
            return

        clusters = self.merge_clusters(clusters)

        if self.get_parameter('debug_clusters').value:
            info = ', '.join(
                f"[d={c['mean_range']:.2f} w={c['width']:.2f} n={c['count']} "
                f"a={math.degrees(c['mid_angle']):.0f}deg]"
                for c in clusters
            )
            self.get_logger().info(f'Clusters({len(clusters)}): {info}',
                                   throttle_duration_sec=1.0)

        clusters = self.reject_robot_clusters(clusters)
        if not clusters:
            if self.last_detection_logged is not False:
                self.get_logger().info('Only robot links detected, no external object')
                self.last_detection_logged = False
            return

        target = max(clusters, key=lambda c: c['count'])

        x_l, y_l = self.cluster_to_cartesian(target)
        point_lidar = PointStamped()
        point_lidar.header.stamp = msg.header.stamp
        point_lidar.header.frame_id = self.get_parameter('lidar_frame').value
        point_lidar.point.x = x_l
        point_lidar.point.y = y_l
        point_lidar.point.z = 0.0

        point_world = self.transform_to_world(point_lidar)
        if point_world is None:
            return

        self.position_pub.publish(point_world)

        if self.get_parameter('publish_marker').value:
            self.publish_marker(point_world, target)

        if self.last_detection_logged is not True:
            self.get_logger().info(
                f'Object detected: dist={target["mean_range"]:.2f}m, '
                f'width={target["width"]:.2f}m, points={target["count"]}'
            )
            self.last_detection_logged = True

    def extract_clusters(self, msg: LaserScan):
        threshold = self.get_parameter('cluster_distance_threshold').value
        min_points = self.get_parameter('min_cluster_points').value
        min_w = self.get_parameter('min_object_width').value
        max_w = self.get_parameter('max_object_width').value
        max_range = self.get_parameter('max_detection_range').value

        clusters = []
        current = []

        for i, r in enumerate(msg.ranges):
            valid = (
                math.isfinite(r)
                and msg.range_min <= r <= msg.range_max
                and r <= max_range
                and r >= self.get_parameter('min_detection_range').value
            )

            if valid:
                if current and abs(r - msg.ranges[current[-1]]) > threshold:
                    self.finalize_cluster(
                        clusters, current, msg, min_points, min_w, max_w
                    )
                    current = []
                current.append(i)
            else:
                if current:
                    self.finalize_cluster(
                        clusters, current, msg, min_points, min_w, max_w
                    )
                    current = []

        if current:
            self.finalize_cluster(clusters, current, msg, min_points, min_w, max_w)

        return clusters

    def finalize_cluster(self, clusters, indices, msg, min_points, min_w, max_w):
        if len(indices) < min_points:
            return

        ranges = [msg.ranges[i] for i in indices]
        mean_range = sum(ranges) / len(ranges)

        angular_span = (indices[-1] - indices[0]) * msg.angle_increment
        width = mean_range * angular_span

        if not (min_w <= width <= max_w):
            return

        mid_idx = indices[len(indices) // 2]
        clusters.append({
            'indices': indices,
            'mean_range': mean_range,
            'width': width,
            'count': len(indices),
            'mid_angle': msg.angle_min + mid_idx * msg.angle_increment,
            'min_range': min(ranges),
        })

    def get_link_position_world(self, link_name):
        """Позиция звена робота в кадре world."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame, link_name, rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
            t = tf.transform.translation
            return (t.x, t.y)
        except Exception:
            return None

    def reject_robot_clusters(self, clusters):
        """Отбрасывает кластеры, совпадающие со звеньями робота."""
        radius = self.get_parameter('robot_exclusion_radius').value
        link_names = self.get_parameter('robot_links').value

        link_positions = []
        for name in link_names:
            pos = self.get_link_position_world(name)
            if pos is not None:
                link_positions.append(pos)

        if not link_positions:
            return clusters

        kept = []
        for c in clusters:
            x_l, y_l = self.cluster_to_cartesian(c)
            point = PointStamped()
            point.header.frame_id = self.get_parameter('lidar_frame').value
            point.point.x = x_l
            point.point.y = y_l
            point.point.z = 0.0

            world_point = self.transform_to_world(point)
            if world_point is None:
                kept.append(c)
                continue

            px, py = world_point.point.x, world_point.point.y
            is_robot = any(
                math.hypot(px - lx, py - ly) <= radius
                for lx, ly in link_positions
            )
            if not is_robot:
                kept.append(c)

        return kept

    def merge_clusters(self, clusters):
        """Склеивает кластеры, близкие по углу и дистанции (разорванный объект)."""
        if len(clusters) < 2:
            return clusters

        angle_gap = self.get_parameter('merge_angle_gap').value
        range_gap = self.get_parameter('merge_range_gap').value

        ordered = sorted(clusters, key=lambda c: c['mid_angle'])
        merged = [ordered[0]]

        for c in ordered[1:]:
            prev = merged[-1]
            close_angle = abs(c['mid_angle'] - prev['mid_angle']) <= angle_gap
            close_range = abs(c['mean_range'] - prev['mean_range']) <= range_gap

            if close_angle and close_range:
                total = prev['count'] + c['count']
                merged[-1] = {
                    'indices': prev['indices'] + c['indices'],
                    'mean_range': (prev['mean_range'] * prev['count']
                                   + c['mean_range'] * c['count']) / total,
                    'width': prev['width'] + c['width']
                             + abs(c['mid_angle'] - prev['mid_angle'])
                             * prev['mean_range'],
                    'count': total,
                    'mid_angle': (prev['mid_angle'] * prev['count']
                                  + c['mid_angle'] * c['count']) / total,
                    'min_range': min(prev['min_range'], c['min_range']),
                }
            else:
                merged.append(c)

        return merged

    @staticmethod
    def cluster_to_cartesian(cluster):
        r = cluster['min_range'] + cluster['width'] / 2.0
        a = cluster['mid_angle']
        return r * math.cos(a), r * math.sin(a)

    def transform_to_world(self, point_in: PointStamped):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame,
                point_in.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
        except Exception as exc:
            self.get_logger().warn(
                f'TF lookup failed ({point_in.header.frame_id} -> '
                f'{self.world_frame}): {exc}',
                throttle_duration_sec=5.0,
            )
            return None

        t = tf.transform.translation
        q = tf.transform.rotation

        px, py, pz = point_in.point.x, point_in.point.y, point_in.point.z
        qx, qy, qz, qw = q.x, q.y, q.z, q.w

        tx = 2.0 * (qy * pz - qz * py)
        ty = 2.0 * (qz * px - qx * pz)
        tz = 2.0 * (qx * py - qy * px)

        rx = px + qw * tx + (qy * tz - qz * ty)
        ry = py + qw * ty + (qz * tx - qx * tz)
        rz = pz + qw * tz + (qx * ty - qy * tx)

        out = PointStamped()
        out.header.stamp = point_in.header.stamp
        out.header.frame_id = self.world_frame
        out.point.x = rx + t.x
        out.point.y = ry + t.y
        out.point.z = rz + t.z
        return out

    def publish_marker(self, point: PointStamped, cluster):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.world_frame
        marker.ns = 'detected_human'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = point.point.x
        marker.pose.position.y = point.point.y
        marker.pose.position.z = 0.875
        marker.pose.orientation.w = 1.0
        marker.scale.x = max(cluster['width'], 0.2)
        marker.scale.y = max(cluster['width'], 0.2)
        marker.scale.z = 1.75
        marker.color.r = 0.0
        marker.color.g = 0.6
        marker.color.b = 1.0
        marker.color.a = 0.4
        marker.lifetime.sec = 1
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = LidarHumanDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

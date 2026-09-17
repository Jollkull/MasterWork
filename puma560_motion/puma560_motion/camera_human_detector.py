import math

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import tf2_ros
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker


class CameraHumanDetector(Node):
    """
    Обнаружение человека по depth-изображению камеры глубины.

    Алгоритм (OpenCV):
      1. Пороговая фильтрация по диапазону глубины
      2. Морфологическое открытие/закрытие для подавления шума
      3. Поиск контуров
      4. Фильтрация контуров по площади и пропорциям
      5. Центр контура + медианная глубина -> 3D-точка (модель pinhole)
      6. Преобразование в кадр world через TF
      7. Отбрасывание детекций, совпадающих со звеньями робота
    """

    def __init__(self):
        super().__init__('camera_human_detector')

        self.declare_parameter('depth_topic', '/camera/depth_image')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('output_topic', '/puma/human_position_camera')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('camera_frame', 'camera_optical_frame')

        # Диапазон глубины, в котором ищем объекты, м
        self.declare_parameter('depth_min', 0.5)
        self.declare_parameter('depth_max', 6.0)
        # Фильтрация по высоте над полом, м (отсекает пол и потолок)
        self.declare_parameter('height_min', 0.25)
        self.declare_parameter('height_max', 2.2)
        # Камера видит переднюю поверхность объекта; центр глубже на радиус
        self.declare_parameter('object_radius', 0.25)
        # Минимальная площадь контура в пикселях
        self.declare_parameter('min_contour_area', 80)
        # Допустимое отношение высоты к ширине контура (человек вытянут вверх)
        self.declare_parameter('min_aspect_ratio', 0.8)
        self.declare_parameter('max_aspect_ratio', 8.0)
        # Размер ядра морфологии
        self.declare_parameter('morph_kernel', 5)
        # Радиус исключения вокруг звеньев робота, м
        self.declare_parameter('robot_exclusion_radius', 0.22)
        # Робот целиком: всё в этом радиусе от базы считается роботом
        self.declare_parameter('robot_base_frame', 'link1')
        self.declare_parameter('robot_footprint_radius', 0.0)
        self.declare_parameter('robot_capsule_radius', 0.25)
        self.declare_parameter(
            'robot_links',
            ['link1', 'link2', 'link3', 'link4', 'link5', 'link6', 'link7', 'tool0'],
        )
        self.declare_parameter('publish_marker', True)
        self.declare_parameter('publish_debug_image', True)

        self.world_frame = self.get_parameter('world_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value

        self.bridge = CvBridge()
        self.camera_info = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self.camera_info_callback, 10,
        )
        self.create_subscription(
            Image, self.get_parameter('depth_topic').value,
            self.depth_callback, 10,
        )

        self.position_pub = self.create_publisher(
            PointStamped, self.get_parameter('output_topic').value, 10
        )
        self.marker_pub = self.create_publisher(
            Marker, '/puma/camera_detected_marker', 10
        )
        self.debug_image_pub = self.create_publisher(
            Image, '/puma/camera_debug_image', 10
        )

        self.last_state = None

        self.get_logger().info(
            'camera_human_detector started. '
            f"depth={self.get_parameter('depth_topic').value} -> "
            f"{self.get_parameter('output_topic').value}"
        )

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg

    def depth_callback(self, msg: Image):
        if self.camera_info is None:
            self.log_state('waiting_info', 'Waiting for camera_info...')
            return

        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as exc:
            self.get_logger().warn(f'cv_bridge conversion failed: {exc}',
                                   throttle_duration_sec=5.0)
            return

        mask = self.build_mask(depth)
        contours = self.find_candidates(mask)

        if self.get_parameter('publish_debug_image').value:
            self.publish_debug(mask, contours)

        if not contours:
            self.log_state('no_object', 'No object in camera view')
            return

        # Самый крупный подходящий контур
        contour = max(contours, key=cv2.contourArea)
        point_cam = self.contour_to_point(contour, depth)
        if point_cam is None:
            return

        point_world = self.transform_to_world(point_cam)
        if point_world is None:
            return

        if self.is_robot(point_world):
            self.log_state('robot_only', 'Detected blob belongs to robot, ignoring')
            return

        self.position_pub.publish(point_world)

        if self.get_parameter('publish_marker').value:
            self.publish_marker(point_world)

        area = cv2.contourArea(contour)
        self.log_state(
            'detected',
            f'Object detected: x={point_world.point.x:.2f} '
            f'y={point_world.point.y:.2f} area={area:.0f}px',
        )

    def build_mask(self, depth):
        d_min = self.get_parameter('depth_min').value
        d_max = self.get_parameter('depth_max').value
        k = self.get_parameter('morph_kernel').value

        # Невалидные значения (inf/nan) -> 0, чтобы не мешали порогу
        clean = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)

        mask = cv2.inRange(clean, d_min, d_max)

        # Отсечение пола и потолка по высоте в кадре world
        height_mask = self.build_height_mask(clean)
        if height_mask is not None:
            mask = cv2.bitwise_and(mask, height_mask)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def build_height_mask(self, depth):
        """Маска пикселей, чья высота над полом попадает в заданный диапазон."""
        if self.camera_info is None:
            return None

        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame, self.camera_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.1),
            )
        except Exception:
            return None

        k = self.camera_info.k
        fx, fy = k[0], k[4]
        cx, cy = k[2], k[5]
        if fx == 0 or fy == 0:
            return None

        h, w = depth.shape
        us, vs = np.meshgrid(np.arange(w), np.arange(h))

        z = depth
        x = (us - cx) * z / fx
        y = (vs - cy) * z / fy

        q = tf.transform.rotation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w

        # Только Z-компонента результата поворота (высота в world)
        tx = 2.0 * (qy * z - qz * y)
        ty = 2.0 * (qz * x - qx * z)
        world_z = (z + qw * (2.0 * (qx * y - qy * x))
                   + (qx * ty - qy * tx) + tf.transform.translation.z)

        h_min = self.get_parameter('height_min').value
        h_max = self.get_parameter('height_max').value

        return ((world_z >= h_min) & (world_z <= h_max)).astype(np.uint8) * 255

    def find_candidates(self, mask):
        min_area = self.get_parameter('min_contour_area').value
        min_ar = self.get_parameter('min_aspect_ratio').value
        max_ar = self.get_parameter('max_aspect_ratio').value

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        result = []
        for c in contours:
            if cv2.contourArea(c) < min_area:
                continue
            x, y, w, h = cv2.boundingRect(c)
            if w == 0:
                continue
            aspect = float(h) / float(w)
            if not (min_ar <= aspect <= max_ar):
                continue
            result.append(c)
        return result

    def contour_to_point(self, contour, depth):
        """Центр контура + медианная глубина -> 3D-точка в кадре камеры."""
        mask = np.zeros(depth.shape, dtype=np.uint8)
        cv2.drawContours(mask, [contour], -1, 255, thickness=cv2.FILLED)

        values = depth[mask == 255]
        values = values[np.isfinite(values)]
        values = values[values > 0]
        if values.size == 0:
            return None

        z = float(np.median(values))

        moments = cv2.moments(contour)
        if abs(moments['m00']) < 1e-6:
            return None
        u = moments['m10'] / moments['m00']
        v = moments['m01'] / moments['m00']

        # Pinhole-модель: K = [fx 0 cx; 0 fy cy; 0 0 1]
        k = self.camera_info.k
        fx, fy = k[0], k[4]
        cx, cy = k[2], k[5]
        if fx == 0 or fy == 0:
            return None

        # Смещаем точку вглубь на радиус объекта вдоль луча зрения
        z += self.get_parameter('object_radius').value

        point = PointStamped()
        point.header.frame_id = self.camera_frame
        point.point.x = (u - cx) * z / fx
        point.point.y = (v - cy) * z / fy
        point.point.z = z
        return point

    def transform_to_world(self, point_in: PointStamped):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame, point_in.header.frame_id,
                rclpy.time.Time(), timeout=Duration(seconds=0.1),
            )
        except Exception as exc:
            self.get_logger().warn(f'TF lookup failed: {exc}',
                                   throttle_duration_sec=5.0)
            return None

        t = tf.transform.translation
        q = tf.transform.rotation

        px, py, pz = point_in.point.x, point_in.point.y, point_in.point.z
        qx, qy, qz, qw = q.x, q.y, q.z, q.w

        tx = 2.0 * (qy * pz - qz * py)
        ty = 2.0 * (qz * px - qx * pz)
        tz = 2.0 * (qx * py - qy * px)

        out = PointStamped()
        out.header.stamp = point_in.header.stamp
        out.header.frame_id = self.world_frame
        out.point.x = px + qw * tx + (qy * tz - qz * ty) + t.x
        out.point.y = py + qw * ty + (qz * tx - qx * tz) + t.y
        out.point.z = pz + qw * tz + (qx * ty - qy * tx) + t.z
        return out

    @staticmethod
    def point_segment_distance(px, py, ax, ay, bx, by):
        """Расстояние от точки до отрезка."""
        vx, vy = bx - ax, by - ay
        wx, wy = px - ax, py - ay
        seg_len_sq = vx * vx + vy * vy
        if seg_len_sq < 1e-9:
            return math.hypot(wx, wy)
        t = max(0.0, min(1.0, (wx * vx + wy * vy) / seg_len_sq))
        cx, cy = ax + t * vx, ay + t * vy
        return math.hypot(px - cx, py - cy)

    def link_world_positions(self):
        result = []
        for name in self.get_parameter('robot_links').value:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.world_frame, name, rclpy.time.Time(),
                    timeout=Duration(seconds=0.05),
                )
                result.append((tf.transform.translation.x,
                               tf.transform.translation.y))
            except Exception:
                continue
        return result

    def is_robot(self, point: PointStamped):
        px, py = point.point.x, point.point.y
        points = self.link_world_positions()
        if not points:
            return False

        radius = self.get_parameter('robot_exclusion_radius').value
        capsule_r = self.get_parameter('robot_capsule_radius').value

        # Капсульная модель: отрезки между соседними звеньями
        for i in range(len(points) - 1):
            a, b = points[i], points[i + 1]
            if self.point_segment_distance(px, py, a[0], a[1],
                                           b[0], b[1]) <= capsule_r:
                return True

        # Дополнительно - окрестности самих звеньев
        for lx, ly in points:
            if math.hypot(px - lx, py - ly) <= radius:
                return True

        return False

    def publish_debug(self, mask, contours):
        vis = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        cv2.drawContours(vis, contours, -1, (0, 255, 0), 2)
        for c in contours:
            x, y, w, h = cv2.boundingRect(c)
            cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 0, 255), 1)
        try:
            self.debug_image_pub.publish(
                self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
            )
        except Exception:
            pass

    def publish_marker(self, point: PointStamped):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.world_frame
        marker.ns = 'camera_detected_human'
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
        marker.color.r = 1.0
        marker.color.g = 0.4
        marker.color.b = 0.0
        marker.color.a = 0.35
        marker.lifetime.sec = 1
        self.marker_pub.publish(marker)

    def log_state(self, state, message):
        if self.last_state != state:
            self.get_logger().info(message)
            self.last_state = state


def main(args=None):
    rclpy.init(args=args)
    node = CameraHumanDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

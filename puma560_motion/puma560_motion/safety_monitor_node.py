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
        # Радиус объекта-человека, м. Детектор даёт центр объекта,
        # а безопасность считается до его поверхности.
        self.declare_parameter('human_radius', 0.25)
        # Человек - вертикально вытянутый объект, поэтому разница высот
        # между звеном и точкой детекции не должна увеличивать дистанцию.
        self.declare_parameter('use_horizontal_distance', True)
        # Упреждающее срабатывание: учитывается не только текущая,
        # но и прогнозируемая позиция человека (ISO/TS 15066)
        self.declare_parameter('use_prediction', True)
        # Если позиция человека не обновляется дольше этого времени,
        # считаем, что он покинул зону наблюдения, с
        self.declare_parameter('human_position_timeout', 1.5)
        self.declare_parameter(
            'predicted_position_topic', '/puma/human_position_predicted'
        )

        # --- Динамический расчёт защитного расстояния (ISO/TS 15066) ---
        # S = Vh*(Tr+Ts) + Vr*Tr + B + C
        self.declare_parameter('use_dynamic_zones', True)
        # Время реакции системы (детекция + принятие решения), с
        self.declare_parameter('reaction_time', 0.25)
        # Время срабатывания тормоза контроллера, с
        self.declare_parameter('brake_time', 0.15)
        # Замедление робота при торможении, м/с^2
        self.declare_parameter('robot_deceleration', 2.0)
        # Суммарная погрешность измерения положения, м
        self.declare_parameter('sensor_uncertainty', 0.08)
        # Во сколько раз зона предупреждения шире зоны остановки
        self.declare_parameter('warn_zone_factor', 2.0)
        # Границы, ниже/выше которых зона не уходит, м
        self.declare_parameter('min_stop_distance', 0.15)
        self.declare_parameter('max_stop_distance', 1.5)
        self.declare_parameter('human_speed_topic', '/puma/human_speed')

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
        self.human_position_time = None
        self.predicted_position = None
        self.human_speed = 0.0
        self.prev_tcp = None
        self.prev_tcp_time = None
        self.robot_speed = 0.0
        self.current_stop_zone = None
        self.current_warn_zone = None
        self.current_mode = 'SAFE'  # SAFE / SLOW / STOP

        # --- Подписки/публикации ---
        self.create_subscription(
            PointStamped, human_topic, self.human_position_callback, 10
        )
        self.create_subscription(
            PointStamped,
            self.get_parameter('predicted_position_topic').value,
            self.predicted_callback, 10,
        )
        self.create_subscription(
            Float64, self.get_parameter('human_speed_topic').value,
            self.human_speed_callback, 10,
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
        self.stop_zone_pub = self.create_publisher(
            Float64, '/puma/stop_zone_radius', 10
        )
        self.warn_zone_pub = self.create_publisher(
            Float64, '/puma/warn_zone_radius', 10
        )
        self.robot_speed_pub = self.create_publisher(
            Float64, '/puma/robot_speed', 10
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
        self.human_position_time = self.get_clock().now().nanoseconds * 1e-9

    def human_speed_callback(self, msg: Float64):
        self.human_speed = max(0.0, msg.data)

    def update_robot_speed(self, tcp_position):
        """Скорость TCP по смещению между тиками."""
        now = self.get_clock().now().nanoseconds * 1e-9

        if self.prev_tcp is None or self.prev_tcp_time is None:
            self.prev_tcp = tcp_position
            self.prev_tcp_time = now
            return

        dt = now - self.prev_tcp_time
        if dt < 1e-3:
            return

        dist = math.sqrt(
            sum((tcp_position[i] - self.prev_tcp[i]) ** 2 for i in range(3))
        )
        raw_speed = dist / dt

        # Сглаживание, чтобы шум TF не давал выбросов
        self.robot_speed = 0.7 * self.robot_speed + 0.3 * raw_speed
        self.prev_tcp = tcp_position
        self.prev_tcp_time = now

    def compute_protective_distance(self):
        """Защитное расстояние по ISO/TS 15066."""
        if not self.get_parameter('use_dynamic_zones').value:
            return self.distance_stop, self.distance_warn

        tr = self.get_parameter('reaction_time').value
        ts = self.get_parameter('brake_time').value
        decel = self.get_parameter('robot_deceleration').value
        c = self.get_parameter('sensor_uncertainty').value

        # Вклад движения человека за время реакции и торможения
        sh = self.human_speed * (tr + ts)
        # Вклад движения робота за время реакции
        sr = self.robot_speed * tr
        # Тормозной путь робота
        ss = (self.robot_speed ** 2) / (2.0 * decel) if decel > 0 else 0.0

        stop_zone = sh + sr + ss + c

        lo = self.get_parameter('min_stop_distance').value
        hi = self.get_parameter('max_stop_distance').value
        stop_zone = max(lo, min(hi, stop_zone))

        warn_zone = stop_zone * self.get_parameter('warn_zone_factor').value
        warn_zone = min(warn_zone, hi * 2.0)

        return stop_zone, warn_zone

    def predicted_callback(self, msg: PointStamped):
        self.predicted_position = (msg.point.x, msg.point.y, msg.point.z)

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

    def distance(self, a, b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        if self.get_parameter('use_horizontal_distance').value:
            d = math.sqrt(dx * dx + dy * dy)
        else:
            dz = a[2] - b[2]
            d = math.sqrt(dx * dx + dy * dy + dz * dz)
        # Дистанция до поверхности объекта, а не до его центра
        d -= self.get_parameter('human_radius').value
        return max(d, 0.0)

    def tick(self):
        link_positions = {}
        for link in self.link_names:
            pos = self.get_link_position(link)
            if pos is not None:
                link_positions[link] = pos

        tcp = link_positions.get(self.get_parameter('status_text_frame').value)
        if tcp is not None:
            self.update_robot_speed(tcp)

        self.publish_link_safety_markers(link_positions)

        # Данные устарели - человек вышел из зоны наблюдения
        if self.human_position is not None and self.human_position_time is not None:
            age = (self.get_clock().now().nanoseconds * 1e-9
                   - self.human_position_time)
            if age > self.get_parameter('human_position_timeout').value:
                self.get_logger().info(
                    f'Human position lost ({age:.1f}s without update), '
                    'returning to SAFE'
                )
                self.human_position = None
                self.human_position_time = None
                self.predicted_position = None

        if self.human_position is None:
            self.publish_mode('SAFE', 1.0, None)
            return

        targets = [self.human_position]
        if (self.get_parameter('use_prediction').value
                and self.predicted_position is not None):
            targets.append(self.predicted_position)

        min_dist = None
        for pos in link_positions.values():
            for target in targets:
                d = self.distance(pos, target)
                if min_dist is None or d < min_dist:
                    min_dist = d

        if min_dist is None:
            return

        stop_zone, warn_zone = self.compute_protective_distance()
        self.current_stop_zone = stop_zone
        self.current_warn_zone = warn_zone

        self.stop_zone_pub.publish(Float64(data=float(stop_zone)))
        self.warn_zone_pub.publish(Float64(data=float(warn_zone)))
        self.robot_speed_pub.publish(Float64(data=float(self.robot_speed)))

        if min_dist <= stop_zone:
            mode, scale = 'STOP', 0.0
        elif min_dist <= warn_zone:
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
        if self.current_stop_zone is not None:
            marker.text += f'\nzone {self.current_stop_zone:.2f}m'

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
            warn_r = (self.current_warn_zone
                      if self.current_warn_zone is not None
                      else self.distance_warn)
            d_warn = warn_r * 2.0
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
            stop_r = (self.current_stop_zone
                      if self.current_stop_zone is not None
                      else self.distance_stop)
            d_stop = stop_r * 2.0
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
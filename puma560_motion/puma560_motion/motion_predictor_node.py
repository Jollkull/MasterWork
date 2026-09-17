import math
from collections import deque

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, Vector3Stamped
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker


class MotionPredictorNode(Node):
    """
    Прогнозирование положения человека на заданный горизонт времени.

    Скорость оценивается методом наименьших квадратов по истории позиций,
    что устойчивее к шуму измерений, чем простая разность соседних точек.
    Прогноз используется системой безопасности для упреждающего срабатывания
    согласно ISO/TS 15066 (учёт скорости сближения оператора).
    """

    def __init__(self):
        super().__init__('motion_predictor_node')

        self.declare_parameter('input_topic', '/puma/human_position')
        self.declare_parameter('predicted_topic', '/puma/human_position_predicted')
        self.declare_parameter('velocity_topic', '/puma/human_velocity')
        self.declare_parameter('speed_topic', '/puma/human_speed')
        self.declare_parameter('world_frame', 'world')

        # Длина истории для оценки скорости
        self.declare_parameter('history_size', 8)
        # На сколько секунд вперёд прогнозируем
        self.declare_parameter('prediction_horizon', 0.8)
        # История старше этого возраста сбрасывается, с
        self.declare_parameter('history_timeout', 1.0)
        # Ограничение скорости сверху (ISO 13855: скорость человека 1.6 м/с)
        self.declare_parameter('max_human_speed', 2.0)
        # Ниже этого порога считаем человека неподвижным, м/с
        self.declare_parameter('min_speed_threshold', 0.05)
        self.declare_parameter('rate_hz', 10.0)

        self.world_frame = self.get_parameter('world_frame').value
        self.history = deque(maxlen=self.get_parameter('history_size').value)

        self.create_subscription(
            PointStamped, self.get_parameter('input_topic').value,
            self.position_callback, 10,
        )

        self.predicted_pub = self.create_publisher(
            PointStamped, self.get_parameter('predicted_topic').value, 10
        )
        self.velocity_pub = self.create_publisher(
            Vector3Stamped, self.get_parameter('velocity_topic').value, 10
        )
        self.speed_pub = self.create_publisher(
            Float64, self.get_parameter('speed_topic').value, 10
        )
        self.marker_pub = self.create_publisher(
            Marker, '/puma/prediction_marker', 10
        )

        rate = self.get_parameter('rate_hz').value
        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.last_state = None

        self.get_logger().info(
            'motion_predictor_node started. horizon='
            f"{self.get_parameter('prediction_horizon').value}s"
        )

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def position_callback(self, msg: PointStamped):
        t = self.now_sec()

        # Разрыв в данных - история больше не отражает движение
        if self.history and (t - self.history[-1][0]) > \
                self.get_parameter('history_timeout').value:
            self.history.clear()

        self.history.append((t, msg.point.x, msg.point.y, msg.point.z))

    def estimate_velocity(self):
        """Скорость методом наименьших квадратов по истории."""
        if len(self.history) < 3:
            return None

        t0 = self.history[0][0]
        times = [h[0] - t0 for h in self.history]
        n = len(times)

        mean_t = sum(times) / n
        denom = sum((t - mean_t) ** 2 for t in times)
        if denom < 1e-9:
            return None

        velocity = []
        for axis in (1, 2, 3):
            values = [h[axis] for h in self.history]
            mean_v = sum(values) / n
            num = sum((times[i] - mean_t) * (values[i] - mean_v)
                      for i in range(n))
            velocity.append(num / denom)

        return velocity

    def tick(self):
        if not self.history:
            self.report('no_data', 'No position data for prediction')
            return

        # Данные устарели
        if (self.now_sec() - self.history[-1][0]) > \
                self.get_parameter('history_timeout').value:
            self.history.clear()
            self.report('stale', 'Position data stale, history cleared')
            return

        velocity = self.estimate_velocity()
        if velocity is None:
            self.report('collecting', 'Collecting history for velocity estimate')
            return

        vx, vy, vz = velocity
        speed = math.hypot(vx, vy)

        # Ограничение сверху отсекает выбросы измерений
        max_speed = self.get_parameter('max_human_speed').value
        if speed > max_speed:
            scale = max_speed / speed
            vx, vy, vz = vx * scale, vy * scale, vz * scale
            speed = max_speed

        # Ниже порога считаем человека стоящим
        if speed < self.get_parameter('min_speed_threshold').value:
            vx = vy = vz = 0.0
            speed = 0.0
            state = 'static'
        else:
            state = 'moving'

        horizon = self.get_parameter('prediction_horizon').value
        _, last_x, last_y, last_z = self.history[-1]

        predicted = PointStamped()
        predicted.header.stamp = self.get_clock().now().to_msg()
        predicted.header.frame_id = self.world_frame
        predicted.point.x = last_x + vx * horizon
        predicted.point.y = last_y + vy * horizon
        predicted.point.z = last_z

        self.predicted_pub.publish(predicted)

        vel_msg = Vector3Stamped()
        vel_msg.header = predicted.header
        vel_msg.vector.x = vx
        vel_msg.vector.y = vy
        vel_msg.vector.z = vz
        self.velocity_pub.publish(vel_msg)

        self.speed_pub.publish(Float64(data=speed))
        self.publish_marker(last_x, last_y, last_z, predicted, speed)

        self.report(state, f'Human {state}, speed={speed:.2f} m/s')

    def publish_marker(self, x, y, z, predicted: PointStamped, speed):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.world_frame
        marker.ns = 'motion_prediction'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        start = marker.points.__class__()
        from geometry_msgs.msg import Point
        p_start = Point()
        p_start.x, p_start.y, p_start.z = x, y, 0.9
        p_end = Point()
        p_end.x, p_end.y, p_end.z = predicted.point.x, predicted.point.y, 0.9
        marker.points = [p_start, p_end]

        marker.scale.x = 0.05   # диаметр древка
        marker.scale.y = 0.12   # диаметр наконечника
        marker.scale.z = 0.15   # длина наконечника

        # Чем быстрее движется - тем краснее
        marker.color.r = min(1.0, speed / 1.5)
        marker.color.g = max(0.0, 1.0 - speed / 1.5)
        marker.color.b = 0.1
        marker.color.a = 0.9
        marker.lifetime.sec = 1
        self.marker_pub.publish(marker)

    def report(self, state, message):
        if state != self.last_state:
            self.get_logger().info(message)
            self.last_state = state


def main(args=None):
    rclpy.init(args=args)
    node = MotionPredictorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

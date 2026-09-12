import csv
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String


class ExperimentLoggerNode(Node):
    """
    Пишет CSV-лог экспериментов: время, дистанция, режим, коэффициент скорости.
    Также фиксирует время реакции - интервал между тем, как дистанция
    пересекла порог, и тем, как система применила новый режим.
    """

    def __init__(self):
        super().__init__('experiment_logger_node')

        self.declare_parameter('output_path', '/root/workspace_saved/experiments/log.csv')
        self.declare_parameter('distance_warn', 0.6)
        self.declare_parameter('distance_stop', 0.3)

        output_path = self.get_parameter('output_path').value
        os.makedirs(os.path.dirname(output_path), exist_ok=True)

        self.distance_warn = self.get_parameter('distance_warn').value
        self.distance_stop = self.get_parameter('distance_stop').value

        self.min_distance = None
        self.safety_status = None
        self.speed_scaling = None

        self.last_mode = None
        self.threshold_cross_time = None  # время последнего пересечения любого порога

        self.file = open(output_path, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            'wall_time', 'min_distance', 'safety_status',
            'speed_scaling', 'reaction_time_s'
        ])

        self.create_subscription(Float64, '/puma/min_distance', self.on_distance, 10)
        self.create_subscription(String, '/puma/safety_status', self.on_status, 10)
        self.create_subscription(
            Float64, '/joint_trajectory_controller/speed_scaling_input',
            self.on_scaling, 10
        )

        self.get_logger().info(f'experiment_logger_node writing to {output_path}')

    def on_distance(self, msg: Float64):
        d = msg.data
        prev = self.min_distance
        self.min_distance = d

        # Любое новое пересечение порога обновляет метку времени -
        # даже если предыдущая ещё не была "использована" в on_status.
        if prev is not None:
            crossed_warn = (prev > self.distance_warn) != (d > self.distance_warn)
            crossed_stop = (prev > self.distance_stop) != (d > self.distance_stop)
            if crossed_warn or crossed_stop:
                self.threshold_cross_time = time.monotonic()

        self.write_row()

    def on_status(self, msg: String):
        new_mode = msg.data
        reaction_time = None

        if new_mode != self.last_mode and self.threshold_cross_time is not None:
            reaction_time = time.monotonic() - self.threshold_cross_time
            self.threshold_cross_time = None  # использовано, ждём следующего пересечения

        self.last_mode = new_mode
        self.safety_status = new_mode
        self.write_row(reaction_time)

    def on_scaling(self, msg: Float64):
        self.speed_scaling = msg.data
        self.write_row()

    def write_row(self, reaction_time=None):
        self.writer.writerow([
            time.time(),
            self.min_distance if self.min_distance is not None else '',
            self.safety_status if self.safety_status is not None else '',
            self.speed_scaling if self.speed_scaling is not None else '',
            reaction_time if reaction_time is not None else '',
        ])
        self.file.flush()

    def destroy_node(self):
        self.file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

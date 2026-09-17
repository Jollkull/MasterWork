import math
import subprocess
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float64
from geometry_msgs.msg import PointStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration as DurationMsg


JOINT_NAMES = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']

ROBOT_PRESETS = {
    'Домашняя': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'Вытянута вперёд': [0.0, 1.5708, -1.5708, 0.0, 0.0, 0.0],
    'Вправо': [1.5708, 1.2, -0.8, 0.0, 0.0, 0.0],
    'Влево': [-1.5708, 1.2, -0.8, 0.0, 0.0, 0.0],
    'Сложена': [0.0, 1.2, 1.2, 0.0, 0.5, 0.0],
    'Наклон вниз': [0.0, 1.4, 0.6, 0.0, 0.8, 0.0],
}


class ControlPanelNode(Node):
    """Графический пульт управления сценой: человек и суставы робота."""

    def __init__(self):
        super().__init__('control_panel_node')

        self.declare_parameter('world_name', 'empty')
        self.declare_parameter('human_models', ['human_dummy', 'human_visual'])
        self.declare_parameter('update_rate_hz', 30.0)

        self.world = self.get_parameter('world_name').value
        self.human_models = list(self.get_parameter('human_models').value)

        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10,
        )

        # Телеметрия для индикаторов
        self.safety_status = '—'
        self.min_distance = None
        self.stop_zone = None
        self.human_speed = None
        self.fusion_source = '—'
        self.human_pos = None

        self.create_subscription(String, '/puma/safety_status',
                                 self._on_status, 10)
        self.create_subscription(Float64, '/puma/min_distance',
                                 self._on_distance, 10)
        self.create_subscription(Float64, '/puma/stop_zone_radius',
                                 self._on_zone, 10)
        self.create_subscription(Float64, '/puma/human_speed',
                                 self._on_speed, 10)
        self.create_subscription(String, '/puma/fusion_source',
                                 self._on_source, 10)
        self.create_subscription(PointStamped, '/puma/human_position',
                                 self._on_position, 10)

        self.motion_thread = None
        self.motion_stop = threading.Event()

        self.get_logger().info('control_panel_node started')

    # --- Приём телеметрии ---

    def _on_status(self, msg): self.safety_status = msg.data
    def _on_distance(self, msg): self.min_distance = msg.data
    def _on_zone(self, msg): self.stop_zone = msg.data
    def _on_speed(self, msg): self.human_speed = msg.data
    def _on_source(self, msg): self.fusion_source = msg.data
    def _on_position(self, msg):
        self.human_pos = (msg.point.x, msg.point.y)

    # --- Управление человеком ---

    def set_human_pose(self, x, y):
        """Мгновенное перемещение обеих моделей человека."""
        for name in self.human_models:
            req = (f'name: "{name}", position: {{x: {x}, y: {y}, z: 0}}, '
                   f'orientation: {{w: 1}}')
            subprocess.run(
                ['gz', 'service', '-s', f'/world/{self.world}/set_pose',
                 '--reqtype', 'gz.msgs.Pose',
                 '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '300', '--req', req],
                capture_output=True,
            )

    def stop_motion(self):
        self.motion_stop.set()
        if self.motion_thread is not None:
            self.motion_thread.join(timeout=1.0)
        self.motion_stop.clear()

    def move_human_smooth(self, x_from, y_from, x_to, y_to, speed, on_done=None):
        """Плавное перемещение с заданной скоростью, м/с."""
        self.stop_motion()

        def worker():
            dist = math.hypot(x_to - x_from, y_to - y_from)
            if dist < 1e-3 or speed <= 0:
                self.set_human_pose(x_to, y_to)
                if on_done:
                    on_done()
                return

            rate = self.get_parameter('update_rate_hz').value
            dt = 1.0 / rate
            steps = max(1, int((dist / speed) / dt))

            for i in range(1, steps + 1):
                if self.motion_stop.is_set():
                    return
                t = i / steps
                self.set_human_pose(
                    x_from + (x_to - x_from) * t,
                    y_from + (y_to - y_from) * t,
                )
                self.motion_stop.wait(dt)

            if on_done:
                on_done()

        self.motion_thread = threading.Thread(target=worker, daemon=True)
        self.motion_thread.start()

    # --- Управление роботом ---

    def send_joints(self, positions, seconds=3.0):
        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = DurationMsg(
            sec=int(seconds),
            nanosec=int((seconds - int(seconds)) * 1e9),
        )
        msg.points = [point]
        self.traj_pub.publish(msg)


class ControlPanelGUI:
    def __init__(self, node: ControlPanelNode):
        self.node = node

        self.root = tk.Tk()
        self.root.title('PUMA 560 — пульт управления сценой')
        self.root.geometry('560x760')

        self.human_x = tk.DoubleVar(value=1.5)
        self.human_y = tk.DoubleVar(value=0.0)
        self.speed = tk.DoubleVar(value=0.5)
        self.joint_vars = [tk.DoubleVar(value=0.0) for _ in JOINT_NAMES]
        self.move_time = tk.DoubleVar(value=3.0)

        self._build()
        self._refresh()

    def _build(self):
        pad = {'padx': 8, 'pady': 4}

        # --- Индикаторы ---
        info = ttk.LabelFrame(self.root, text='Состояние системы')
        info.pack(fill='x', **pad)

        self.status_label = tk.Label(
            info, text='—', font=('Sans', 20, 'bold'), fg='gray'
        )
        self.status_label.pack(pady=6)

        self.detail_label = tk.Label(info, text='', font=('Sans', 10))
        self.detail_label.pack(pady=2)

        # --- Человек ---
        human = ttk.LabelFrame(self.root, text='Положение человека')
        human.pack(fill='x', **pad)

        self._slider(human, 'X, м', self.human_x, -3.0, 4.0)
        self._slider(human, 'Y, м', self.human_y, -3.0, 3.0)
        self._slider(human, 'Скорость, м/с', self.speed, 0.1, 2.0)

        btns = tk.Frame(human)
        btns.pack(fill='x', pady=4)
        tk.Button(btns, text='Переместить мгновенно',
                  command=self.teleport_human).pack(side='left', padx=4)
        tk.Button(btns, text='Идти плавно', bg='#cce5ff',
                  command=self.walk_human).pack(side='left', padx=4)
        tk.Button(btns, text='Стоп',
                  command=self.node.stop_motion).pack(side='left', padx=4)

        presets = tk.Frame(human)
        presets.pack(fill='x', pady=4)
        tk.Label(presets, text='Подойти на:').pack(side='left', padx=4)
        for d in (0.5, 1.0, 1.5, 2.0, 2.5):
            tk.Button(presets, text=f'{d} м', width=5,
                      command=lambda v=d: self.approach(v)).pack(side='left', padx=2)

        # --- Робот ---
        robot = ttk.LabelFrame(self.root, text='Суставы робота, рад')
        robot.pack(fill='both', expand=True, **pad)

        for i, name in enumerate(JOINT_NAMES):
            self._slider(robot, name, self.joint_vars[i], -3.14, 3.14)

        self._slider(robot, 'Время движения, с', self.move_time, 1.0, 10.0)

        rbtns = tk.Frame(robot)
        rbtns.pack(fill='x', pady=4)
        tk.Button(rbtns, text='Выполнить', bg='#cce5ff',
                  command=self.send_joints).pack(side='left', padx=4)
        tk.Button(rbtns, text='Обнулить',
                  command=self.reset_joints).pack(side='left', padx=4)

        pos_frame = tk.Frame(robot)
        pos_frame.pack(fill='x', pady=4)
        tk.Label(pos_frame, text='Готовые позы:').pack(anchor='w', padx=4)
        grid = tk.Frame(pos_frame)
        grid.pack(fill='x')
        for idx, (label, values) in enumerate(ROBOT_PRESETS.items()):
            tk.Button(grid, text=label, width=16,
                      command=lambda v=values: self.apply_preset(v)
                      ).grid(row=idx // 3, column=idx % 3, padx=3, pady=3)

    def _slider(self, parent, label, var, lo, hi):
        frame = tk.Frame(parent)
        frame.pack(fill='x', padx=6, pady=1)
        tk.Label(frame, text=label, width=18, anchor='w').pack(side='left')
        value_label = tk.Label(frame, width=6, anchor='e')
        value_label.pack(side='right')
        scale = tk.Scale(frame, variable=var, from_=lo, to=hi,
                         resolution=0.01, orient='horizontal',
                         showvalue=False, length=280)
        scale.pack(side='right', fill='x', expand=True)

        def update(*_):
            value_label.config(text=f'{var.get():.2f}')
        var.trace_add('write', update)
        update()

    # --- Действия ---

    def teleport_human(self):
        self.node.set_human_pose(self.human_x.get(), self.human_y.get())

    def walk_human(self):
        current = self.node.human_pos
        start_x, start_y = current if current else (3.0, 0.0)
        self.node.move_human_smooth(
            start_x, start_y,
            self.human_x.get(), self.human_y.get(),
            self.speed.get(),
        )

    def approach(self, distance):
        self.human_x.set(distance)
        self.human_y.set(0.0)
        self.walk_human()

    def send_joints(self):
        self.node.send_joints(
            [v.get() for v in self.joint_vars],
            self.move_time.get(),
        )

    def reset_joints(self):
        for v in self.joint_vars:
            v.set(0.0)
        self.send_joints()

    def apply_preset(self, values):
        for var, value in zip(self.joint_vars, values):
            var.set(value)
        self.send_joints()

    # --- Обновление индикаторов ---

    def _refresh(self):
        status = self.node.safety_status
        colors = {'SAFE': '#1a9641', 'SLOW': '#e08214', 'STOP': '#d7191c'}
        self.status_label.config(text=status, fg=colors.get(status, 'gray'))

        parts = []
        if self.node.min_distance is not None:
            parts.append(f'дистанция {self.node.min_distance:.2f} м')
        if self.node.stop_zone is not None:
            parts.append(f'зона {self.node.stop_zone:.2f} м')
        if self.node.human_speed is not None:
            parts.append(f'скорость {self.node.human_speed:.2f} м/с')
        parts.append(f'источник {self.node.fusion_source}')
        self.detail_label.config(text='   |   '.join(parts))

        self.root.after(200, self._refresh)

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = ControlPanelNode()

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    )
    spin_thread.start()

    gui = ControlPanelGUI(node)
    try:
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motion()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

from setuptools import find_packages, setup

package_name = 'puma560_motion'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Файлы лаунчей
        ('share/' + package_name + '/launch', [
            'launch/safety_monitor.launch.py',
            'launch/full_system.launch.py',
        ]),
        # Конфиг RViz
        ('share/' + package_name + '/config', ['config/full_system.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jollkull',
    maintainer_email='shpota.vladislav@gmail.com',
    description='Safety monitor for PUMA560 human-robot shared workspace',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_panel_node = puma560_motion.control_panel_node:main',
            'camera_human_detector = puma560_motion.camera_human_detector:main',
            'lidar_human_detector = puma560_motion.lidar_human_detector:main',
            'safety_monitor_node = puma560_motion.safety_monitor_node:main',
            'human_position_stub = puma560_motion.human_position_stub:main',
            'experiment_logger_node = puma560_motion.experiment_logger_node:main',
            'sensor_fusion_node = puma560_motion.sensor_fusion_node:main',
            'motion_predictor_node = puma560_motion.motion_predictor_node:main',
        ],
    },
)

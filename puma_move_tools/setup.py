from setuptools import find_packages, setup

package_name = 'puma_move_tools'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jollkull',
    maintainer_email='shpota.vladislav@gmail.com',
    description='package for moving topics PUMA560',
    license='MIT',
    entry_points={
        'console_scripts': [
            'puma_pose_to_joint_bridge = puma_move_tools.puma_pose_to_joint_bridge:main',
            'puma_rviz_target_publisher = puma_move_tools.puma_rviz_target_publisher:main',
            'puma_rviz_interactive_target = puma_move_tools.puma_rviz_interactive_target:main',
        ],
    },
)

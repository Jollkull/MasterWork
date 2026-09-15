from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'puma560_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Файлы лаунчей
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # URDF и Xacro
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        # Модели/меши (вложенные папки)
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl', recursive=True)),
        # Конфиги
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Миры Gazebo
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        # SDF-модели объектов сцены
        (os.path.join('share', package_name, 'models', 'human_dummy'),
            glob('models/human_dummy/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)

from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'my_ros2_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # copy file model.sdf, model.config
        (os.path.join('share', package_name, 'models/simple_robot'), glob('models/simple_robot/*.*')),
        (os.path.join('share', package_name, 'models/simple_robot/meshes/collision'), glob('models/simple_robot/meshes/collision/*')),
        (os.path.join('share', package_name, 'models/simple_robot/meshes/visual'), glob('models/simple_robot/meshes/visual/*')),
        (os.path.join('share', package_name, 'models/simple_robot/thumbnails'), glob('models/simple_robot/thumbnails/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pc',
    maintainer_email='pc@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'controller = my_ros2_package.controller:main'
        ],
    },
)

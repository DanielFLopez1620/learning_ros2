from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'm02_ros2_with_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dan1620',
    maintainer_email='dfelipe.lopez@gmail.com',
    description='A package for learning ROS2 Communication with rclpy',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_node = m02_ros2_with_py.hello_node:main',
            'simple_turtle_mov = m02_ros2_with_py.simple_turtle_mov:main',
            'pub_int64 = m02_ros2_with_py.int64_pub:main',
            'sub_int64 = m02_ros2_with_py.int64_sub:main',
            'add_nums_cli = m02_ros2_with_py.add_two_nums_cli:main', 
            'add_nums_srv = m02_ros2_with_py.add_two_nums_srv:main',
            'set_bool_srv = m02_ros2_with_py.set_bool_srv:main',
            'set_bool_cli = m02_ros2_with_py.set_bool_cli:main',
            'saying_hi = m02_ros2_with_py.saying_hi:main',
            'turtle_challenge = m02_ros2_with_py.turtle_challenge:main',
            'turtle_action_srv = m02_ros2_with_py.turtle_action_srv:main',
            'turtle_action_cli = m02_ros2_with_py.turtle_action_cli:main'
        ],
    },
)

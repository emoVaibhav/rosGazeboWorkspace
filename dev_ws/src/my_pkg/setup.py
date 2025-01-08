import os
from glob import glob

from setuptools import setup

package_name = 'my_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'description'), glob('description/*.xacro')),
        (os.path.join('share', package_name, 'description'), glob('description/*.sdf')),
        (os.path.join('share', package_name, 'description', 'partials'), glob('description/partials/*.xacro')),
        (os.path.join('share', package_name, 'description', 'worlds'), glob('description/worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='faaizz',
    maintainer_email='fr33ziey@gmail.com',
    description='ROS 2 Turotials from docs.ros.org',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_pkg.my_node:main',
            'talker = my_pkg.publisher_member_function:main',
            'listener = my_pkg.subscriber_member_function:main',
            'add_service = my_pkg.service_member_function:main',
            'add_client = my_pkg.client_member_function:main',
            'param_node = my_pkg.parameters_node:main',
            'static_tf2_broadcaster = my_pkg.static_tf2_broadcaster:main',
            'tf2_listener = my_pkg.tf2_listener:main',
            'frame_tf2_broadcaster = my_pkg.frame_tf2_broadcaster:main',
            'pose_subscriber = my_pkg.pose_subscriber:main',
            'tf2_dist_calculator = my_pkg.tf2_dist_calculator:main',
            'img_saver = my_pkg.img_saver:main',
            'Car_control = my_pkg.Car_control:main',
            'Car_control_UGV = my_pkg.car_control_copy:main',
        ],
    },
)

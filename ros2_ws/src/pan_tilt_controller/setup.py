from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pan_tilt_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rndmtrsh',
    maintainer_email='t.maulanaakmal@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # "test_node = pan_tilt_controller.first_node:main",
            # "draw_circle = pan_tilt_controller.draw_circle:main",
            # "pose_subscriber = pan_tilt_controller.pose_subscriber:main",
            # "turtle_controller = pan_tilt_controller.turtle_controller:main",
            "camera_vision_node = pan_tilt_controller.camera_vision_node:main",
            "pid_node = pan_tilt_controller.pid_node:main",
            "serial_node = pan_tilt_controller.serial_node:main",
        ],
    },
)

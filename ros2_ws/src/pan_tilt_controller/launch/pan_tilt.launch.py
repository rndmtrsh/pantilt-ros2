from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pan_tilt_controller',
            executable='camera_vision_node',
            name='camera_vision',
            parameters=[{
                'camera_id': 0,
                'frame_width': 640,
                'frame_height': 480,
                'deadzone': 40,
                'show_debug': True
            }],
            output='screen'
        ),
        Node(
            package='pan_tilt_controller',
            executable='pid_node',
            name='pid',
            parameters=[{
                'Kp': 0.5,
                'Ki': 0.01,
                'Kd': 0.1,
                'max_vel': 2000,
                'rate_limit': 200
            }],
            output='screen'
        ),
        Node(
            package='pan_tilt_controller',
            executable='serial_node',
            name='serial',
            parameters=[{
                'port': '/dev/ttyACM0',
                'baudrate': 115200
            }],
            output='screen'
        ),
    ])
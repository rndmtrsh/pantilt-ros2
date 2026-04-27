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
                'deadzone': 0,
                'show_debug': True
            }],
            output='screen'
        ),
        Node(
            package='pan_tilt_controller',
            executable='pid_node',
            name='pid',
            parameters=[{
                'Kp': 3.0,
                'Ki': 0.0,
                'Kd': 0.0,
                'max_vel': 5000,
                'rate_limit': 1000
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
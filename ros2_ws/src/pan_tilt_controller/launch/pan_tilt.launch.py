from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    enable_logger = LaunchConfiguration('enable_logger')
    show_debug = LaunchConfiguration('show_debug')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_logger',
            default_value='false',
            description='Enable metrics_logger node'
        ),
        DeclareLaunchArgument(
            'show_debug',
            default_value='true',
            description='Show OpenCV debug window'
        ),
        Node(
            package='pan_tilt_controller',
            executable='camera_vision_node',
            name='camera_vision',
            parameters=[{
                'camera_id': 0,
                'frame_width': 640,
                'frame_height': 480,
                'deadzone': 0,
                'show_debug': ParameterValue(show_debug, value_type=bool)
            }],
            output='screen'
        ),
        Node(
            package='pan_tilt_controller',
            executable='metrics_logger',
            name='metrics_logger',
            parameters=[{
                'test_id': 'dist1p5_bright',
                'distance': 1.5,
                'light_condition': 'bright',
                'output_dir': '/home/akmal/Documents/finalproject/metrics_logs',
                'latency_msg_type': ''
            }],
            condition=IfCondition(enable_logger),
            output='screen'
        ),
        Node(
            package='pan_tilt_controller',
            executable='pid_node',
            name='pid',
            parameters=[{
                'Kp': 7.0,
                'Ki': 0.8,
                'Kd': 1.5,
                'max_vel': 5000,
                'rate_limit': 1000,
                'control_rate': 20
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
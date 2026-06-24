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
            description='Show OpenCV GUI window'
        ),

        Node(
            package='pan_tilt_controller',
            executable='camera_vision_node',
            name='camera_vision',
            parameters=[{
                'camera_id': 0,
                'frame_width': 854,
                'frame_height': 480,
                'max_jump': 500,
                'deadzone_h': 0.50,             # horizontal deadzone 
                'deadzone_v': 0.40,             # vertical deadzone 
                'capture_rate': 24.0,           # opencv rate
                'inference_rate': 10.0,         # YOLO inference rate,
                'vertical_ref_ratio': 0.33,
                'show_debug': ParameterValue(show_debug, value_type=bool),
                'flip_horizontal': True,
            }],
            output='screen'
        ),
        Node(
            package='pan_tilt_controller',
            executable='pid_node',
            name='pid',
            parameters=[{
                'Kp': 2.1,
                'Ki': 0.8,
                'Kd': 1.2,
                'max_vel': 5000,
                'accel_limit': 1000.0,
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
        Node(
            package='pan_tilt_controller',
            executable='metrics_logger',
            name='metrics_logger',
            parameters=[{
                'test_id': 'dist1p5_bright',
                'distance': 1.5,
                'light_condition': 'bright',
                'output_dir': '/home/akmal/Documents/finalproject/metrics_logs',
                'latency_total_topic': '/latency/control_serial',
                'latency_control_topic': '/latency/control_compute',
                'latency_serial_topic': '/latency/serial_write'
            }],
            condition=IfCondition(enable_logger),
            output='screen'
        ),
    ])
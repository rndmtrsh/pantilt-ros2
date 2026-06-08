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
        DeclareLaunchArgument(
            'deadzone',
            default_value='0.50',           # 50% dari dimensi frame
            description='Rasio comfort zone (0=mati, 0.5=50% dari lebar/tinggi)'
        ),
        DeclareLaunchArgument(
            'inference_rate',
            default_value='10.0',           # Hz, YOLO hanya dijalankan 10x per detik
            description='Batas frekuensi inferensi YOLO (Hz, 0=tidak terbatas)'
        ),

        Node(
            package='pan_tilt_controller',
            executable='camera_vision_node',
            name='camera_vision',
            parameters=[{
                'camera_id': 0,
                'frame_width': 854,
                'frame_height': 480,
                'deadzone': LaunchConfiguration('deadzone'),
                'inference_rate': LaunchConfiguration('inference_rate'),
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
                'Kp': 0.7,
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
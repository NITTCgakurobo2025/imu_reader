from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    name_parameter = DeclareLaunchArgument(
        'robot_name',
        default_value='R1')

    name = LaunchConfiguration('robot_name')

    return LaunchDescription([
        name_parameter,
        Node(
            package='imu_reader',
            executable='imu_reader',
            namespace=name,
            name='imu_reader',
            output='screen',
            parameters=[{
                'output_topic': 'imu',
                'frame_id': [name, '/imu_link'],
            }]
        )
    ])

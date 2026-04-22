from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='limelight_perception',
            executable='limelight_publisher',
            name='limelight_publisher',
            output='screen',
            parameters=['config/params.yaml'],
        ),
    ])

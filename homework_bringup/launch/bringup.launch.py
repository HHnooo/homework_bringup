import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('homework_bringup'),
        'config',
        'params.yaml'
    )

    serial_arg = DeclareLaunchArgument(
        'serial',
        default_value='/dev/pts/1',
        description='Serial port device path'
    )

    node = Node(
        package='homework_bringup',
        executable='main_node',
        name='main_node',
        parameters=[config, {'serial_port': LaunchConfiguration('serial')}],
        output='screen'
    )

    return LaunchDescription([
        serial_arg,
        node
    ])
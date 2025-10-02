from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('color_roi_detector'),
        'config',
        'default.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=cfg),
        Node(
            package='color_roi_detector',
            executable='color_roi_node',
            name='color_roi_node',
            output='screen',
            parameters=[LaunchConfiguration('config')],
        ),
    ])

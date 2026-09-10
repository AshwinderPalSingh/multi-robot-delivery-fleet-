import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    slam_params_file = LaunchConfiguration('slam_params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(
                get_package_share_directory('hospital_robot_description'),
                'config', 'slam_toolbox_params.yaml'),
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params_file],
        ),
    ])

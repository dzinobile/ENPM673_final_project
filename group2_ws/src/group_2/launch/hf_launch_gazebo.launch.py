from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "debug",
            default_value="False",  
            description="Enable debug mode"
        ),
        # Node configuration
        Node(
            package='group_2',
            executable='horizon_finder_gazebo.py',
            output='screen',
            parameters=[
                {"debug": LaunchConfiguration("debug")},
            ]
        )
    ])

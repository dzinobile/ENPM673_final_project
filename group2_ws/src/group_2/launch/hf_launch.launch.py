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
            executable='horizon_finder.py',
            output='screen',
            parameters=[
                {"debug": LaunchConfiguration("debug")},
            ]
        )
    ])















########### Old code for reference#########################

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         Declare launch arguments (command-line overrides)
#         DeclareLaunchArgument(
#             "horizon_y",
#             default_value="125",  
#             description="Initial horizon Y value"
#         ),
#         DeclareLaunchArgument(
#             "hough_threshold",
#             default_value="100",
#             description="Hough Transform vote threshold"
#         ),
#         DeclareLaunchArgument(
#             "hough_angle_resolution",
#             default_value="0.0174533",  # np.pi/180 as float string
#             description="Hough Transform angle resolution in radians"
#         ),
#         DeclareLaunchArgument(
#             "debug",
#             default_value="False",  
#             description="Enable debug mode"
#         ),
#         # Node configuration
#         Node(
#             package='group_2',
#             executable='horizon_finder.py',
#             output='screen',
#             parameters=[
#                 {"horizon_y": LaunchConfiguration("horizon_y")},
#                 {"hough_threshold": LaunchConfiguration("hough_threshold")},
#                 {"hough_angle_resolution": LaunchConfiguration("hough_angle_resolution")},
#                 {"debug": LaunchConfiguration("debug")},
#             ]
#         )
#     ])
